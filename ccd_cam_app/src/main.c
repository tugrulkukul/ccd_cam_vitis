
#include "xparameters.h"
#include "xgpio.h"
#include "sleep.h"
#include "xspips.h"
#include "xscugic.h"
//#include "xaxidma.h"
#include "xaxivdma.h"
#include "xil_cache.h"
//udp related includes
#include "lwip/udp.h"
#include "netif/xadapter.h"
#include "xscutimer.h"


#define MEM_BASE_ADDR		(0x20000000)
#define MEM_BASE_ADDR2		(0x30000000)
#define UDP_PAYLOAD_LENGTH	(600 + 1) //1byte packet index + 8000 byte data
#define UDP_SEND_PORT		(34567)
#define ETH_LINK_DETECT_INTERVAL 4
#define RESET_RX_CNTR_LIMIT	400

typedef struct
{
	u16 red;
	u16 green;
	u16 blue;
}pixel_data;


XGpio gpio_led0;
XGpio gpio_lm;
XSpiPs spi_lm98725;
XAxiVdma vdma_afe1;
XAxiVdma vdma_afe2;
XScuGic Intc;
static XScuTimer TimerInstance;

pixel_data fb1[2742];

static struct netif server_netif;
struct netif *netif;
struct udp_pcb *udp_1;
struct pbuf *p;
static int ResetRxCntr = 0;

/* missing declaration in lwIP */
void lwip_init();

int init_spi(void)
{
	XSpiPs_Config *spi_lm;
	int status;

	spi_lm = XSpiPs_LookupConfig(XPAR_PS7_SPI_0_DEVICE_ID);
	status = XSpiPs_CfgInitialize(&spi_lm98725, spi_lm, XPAR_PS7_SPI_0_BASEADDR);
	if(status != XST_SUCCESS)
		return status; //something went wrong
	status = XSpiPs_SelfTest(&spi_lm98725);
	if (status != XST_SUCCESS)
		return status;

	XSpiPs_SetClkPrescaler(&spi_lm98725, XSPIPS_CLK_PRESCALE_16);
	status = XSpiPs_SetOptions(&spi_lm98725, XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION);
	if(status != XST_SUCCESS)
		return status; //something went wrong

	return XST_SUCCESS;
}

/*
 * device 0 is lm98725_A, 1 is lm98725_B
 * reg should be lower than 0x20
 * returns the content of the register
 */
u8 read_afe(u8 device, u8 reg)
{
	u8 send[2],recv[2];
	u8 ret_val;

	if(reg > 0x1F)
		return XST_FAILURE;//out of range

	send[0] = reg | 0x80;
	send[1] = 0x00;
	recv[0] = 0;
	recv[1] = 0;

	XSpiPs_SetSlaveSelect(&spi_lm98725, device); //select the correct slave
	XSpiPs_PolledTransfer(&spi_lm98725, send, NULL, 2); //request the register
	send[0] = 0x00;
	XSpiPs_PolledTransfer(&spi_lm98725, send, recv, 2); //read the register value
	XSpiPs_SetSlaveSelect(&spi_lm98725, 0x0F); // explicitly deselect the slave

	ret_val = (recv[0] << 1) | (recv[1] >> 7); //format the register
	return ret_val;
}

/*
 * device 0 is lm98725_A, 1 is lm98725_B
 * reg should be lower than 0x20
 */
void write_afe(u8 device, u8 reg, u8 val)
{
	u8 send[2];

	if(reg > 0x1F)
		return;//out of range

	send[0] = reg;
	send[1] = val;

	XSpiPs_SetSlaveSelect(&spi_lm98725, device); //select the correct slave
	XSpiPs_PolledTransfer(&spi_lm98725, send, NULL, 2); //write to the register
	XSpiPs_SetSlaveSelect(&spi_lm98725, 0x0F); // explicitly deselect the slave

	usleep(50);
	return;
}

void init_heartbeat(void)
{
	XGpio_Config *gpio_led0_config;
	int status;

	gpio_led0_config = XGpio_LookupConfig(XPAR_GPIO_1_DEVICE_ID);
	status = XGpio_CfgInitialize(&gpio_led0, gpio_led0_config, gpio_led0_config->BaseAddress);
	if(status != XST_SUCCESS)
	{
		return; //something went wrong
	}

	XGpio_SetDataDirection(&gpio_led0, 1, 0);
}

void afe_config(int ic_num)
{
	//babishov
		// page 0

		write_afe(ic_num, 0, 0x00);
		write_afe(ic_num, 1, 0x40);
		write_afe(ic_num, 2, 0xC2);
		write_afe(ic_num, 3, 0x46);
		write_afe(ic_num, 4, 0x08);
		write_afe(ic_num, 5, 0x40);
		write_afe(ic_num, 6, 0x00); //no test data
		write_afe(ic_num, 8, 0x04);
		write_afe(ic_num, 9, 0x14);
		write_afe(ic_num, 10, 0x04);
		write_afe(ic_num, 11, 0x14);
		write_afe(ic_num, 12, 0x04);
		write_afe(ic_num, 13, 0x14);
		write_afe(ic_num, 14, 0x18);
		write_afe(ic_num, 15, 0x28);
		write_afe(ic_num, 16, 0x18);
		write_afe(ic_num, 17, 0x28);
		write_afe(ic_num, 18, 0x18);
		write_afe(ic_num, 19, 0x28);
		write_afe(ic_num, 20, 0x00);
		write_afe(ic_num, 21, 0x00);
		write_afe(ic_num, 31, 0x01); // go to the page 1

		// page 1
		write_afe(ic_num, 0, 0x00);
		write_afe(ic_num, 1, 0x61);
		write_afe(ic_num, 2, 0x61);
		write_afe(ic_num, 3, 0x61);
		write_afe(ic_num, 4, 0x90);
		write_afe(ic_num, 5, 0x00);
		write_afe(ic_num, 6, 0x90);
		write_afe(ic_num, 7, 0x00);
		write_afe(ic_num, 8, 0x76);
		write_afe(ic_num, 9, 0x00);
		write_afe(ic_num, 10, 0x90);
		write_afe(ic_num, 11, 0x00);
		write_afe(ic_num, 12, 0x90);
		write_afe(ic_num, 13, 0x00);
		write_afe(ic_num, 14, 0x76);
		write_afe(ic_num, 15, 0x00);
		write_afe(ic_num, 31, 0x02); // to page 2

		// page 2
	//	write_afe(ic_num, 0, 0x00); //ddac disable, black calibration only
	//	write_afe(ic_num, 1, 0xFF); //infinite black calibration
	//	write_afe(ic_num, 2, 0x0D); //ddac scaling linear, adac 1/8 scaling, 8 pixels averaged in black loop
	//	write_afe(ic_num, 3, 0x00); //black calibration target is 0
	//	write_afe(ic_num, 6, 0x01); //autoclpin start at 	px1
	//	write_afe(ic_num, 7, 0x02); //autoclpin end at 		px2
	//	write_afe(ic_num, 8, 0x06); //black loop start at 	px8
	//	write_afe(ic_num, 9, 0x00); //active pixel start at msb
	//	write_afe(ic_num, 10, 0x10); //active pixel start at lsb (0x0010)
	//	write_afe(ic_num, 11, 0x08); //active pixel end at msb
	//	write_afe(ic_num, 12, 0x42); //active pixel end at lsb (0x0842)
	//	write_afe(ic_num, 13, 0x08); //total line length msb
	//	write_afe(ic_num, 14, 0x78); //total line length lsb (0x0878)

		write_afe(ic_num, 0x00, 0x00); //only black calibration
		write_afe(ic_num, 1, 0xFF); //infinite black calibration
		write_afe(ic_num, 2, 0x0F); //ddac scaling linear, adac 1/8 scaling, 32 pixels averaged in black loop
		write_afe(ic_num, 3, 0x00); //black calibration target is 0
		write_afe(ic_num, 0x06, 13); //#of pixel from sh interval end to start of black pixel clamping (0xD = 13)
		write_afe(ic_num, 0x07, 26); //#of pixel from sh interval end to end of black pixel clamping (0x3D = 26)
		write_afe(ic_num, 0x08, 29); //black loop start at px29
		write_afe(ic_num, 0x09, 0x00); //msb#of pixel from sh interval end to first valid data
		write_afe(ic_num, 0x0A, 0x40); //lsb#of pixel from sh interval end to first valid data (0x0040 = 64)
		write_afe(ic_num, 0x0B, 0x0A); //msb#of pixel from sh interval end to last valid data
		write_afe(ic_num, 0x0C, 0xCC); //lsb#of pixel from sh interval end to last valid data (0x0ACC = 2764)
		write_afe(ic_num, 0x0D, 0x0A); //msb line length
		write_afe(ic_num, 0x0E, 0xF2); //lsb line length (0x0AD4=2772) + 10(interval3) + 20(interval1,2)

		write_afe(ic_num, 27, 0x00); //enable pll
		write_afe(ic_num, 28, 0x00); //pll M=0 + 1
		write_afe(ic_num, 29, 0x00); //pll N=0 + 1
		//25/3 output
		//25/9mhz pixel clk

		write_afe(ic_num, 31, 0x03); // to page3

		// page 3
		//write_afe(ic_num, 0, 0x14); //set sh interval 0 length 20
		//write_afe(ic_num, 1, 0x0A); //set sh interval 1 length 10
		//write_afe(ic_num, 2, 0x0A); //set sh interval 2 length 10
		//write_afe(ic_num, 3, 0x00); //set sh interval 3 length 0

		write_afe(ic_num, 0, 0x0A); //set sh interval 0 length 10
		write_afe(ic_num, 1, 0x0A); //set sh interval 1 length 10
		write_afe(ic_num, 2, 0x0A); //set sh interval 2 length 10
		write_afe(ic_num, 31, 0x04); // to page 4

		// page 4
		//write_afe(ic_num, 0, 0x01); //phic1 is high in sh interval 0
		//write_afe(ic_num, 1, 0x01); //phic1 is high in sh interval 1
		//write_afe(ic_num, 2, 0x01); //phic1 is high in sh interval 2
		write_afe(ic_num, 0, 0x11); //phia1&phic1 is high in sh interval 0
		write_afe(ic_num, 1, 0x11); //phia1&phic1 is high in sh interval 1
		write_afe(ic_num, 2, 0x11); //phia1&phic1 is high in sh interval 2
		write_afe(ic_num, 31, 0x05); // to page 5

		// page 5
		//write_afe(ic_num, 0, 0x18); //sh5,sh4 are high in sh interval 0
		//write_afe(ic_num, 1, 0x0C); //sh4,sh3 are high in sh interval 1
		//write_afe(ic_num, 2, 0x04); //sh3 is high in sh interval 2
		write_afe(ic_num, 0, 0x00); //all sh outputs are low in sh interval 0
		write_afe(ic_num, 1, 0x1F); //all sh outputs are high in sh interval 1
		write_afe(ic_num, 2, 0x00); //all sh outputs are low in sh interval 2
		write_afe(ic_num, 31, 0x06); // to page 6

		// page 6
		//phia output 0x0000001FFFFF ___________________________|-------------------|
		//phib output 0x0000001FFFFF ___________________________|-------------------|
		//phic output 0x0000001FFFFF ___________________________|-------------------|
		//rs output   0x001E00000000 ___________|--|_________________________________
		//cp output   0x0003F8000000 ______________|-----|___________________________
		write_afe(ic_num, 0, 0x00); //phia output 0x0000001FFFFF
		write_afe(ic_num, 1, 0x00);
		write_afe(ic_num, 2, 0x00);
		write_afe(ic_num, 3, 0x1F);
		write_afe(ic_num, 4, 0xFF);
		write_afe(ic_num, 5, 0xFF);
		write_afe(ic_num, 6, 0x00); //phib output 0x0000001FFFFF
		write_afe(ic_num, 7, 0x00);
		write_afe(ic_num, 8, 0x00);
		write_afe(ic_num, 9, 0x1F);
		write_afe(ic_num, 10, 0xFF);
		write_afe(ic_num, 11, 0xFF);
		write_afe(ic_num, 12, 0x00); //phic output 0x0000001FFFFF
		write_afe(ic_num, 13, 0x00);
		write_afe(ic_num, 14, 0x00);
		write_afe(ic_num, 15, 0x1F);
		write_afe(ic_num, 16, 0xFF);
		write_afe(ic_num, 17, 0xFF);
		write_afe(ic_num, 18, 0x00); //rs output 0x01FC00000000
		write_afe(ic_num, 19, 0x1E);
		write_afe(ic_num, 20, 0x00);
		write_afe(ic_num, 21, 0x00);
		write_afe(ic_num, 22, 0x00);
		write_afe(ic_num, 23, 0x00);
		write_afe(ic_num, 24, 0x00); //cp output 0x0001FF000000
		write_afe(ic_num, 25, 0x03);
		write_afe(ic_num, 26, 0xF8);
		write_afe(ic_num, 27, 0x00);
		write_afe(ic_num, 28, 0x00);
		write_afe(ic_num, 29, 0x00);
		write_afe(ic_num, 31, 0x07); // to page 7

		// page 7
		write_afe(ic_num, 0, 0x0A); //msb of Pixel count where sh1 goes low to high
		write_afe(ic_num, 1, 0xD4); //lsb of Pixel count where sh1 goes low to high (0x0AD4=2772)
		write_afe(ic_num, 4, 0x0A); //msb of Pixel count where sh2 goes low to high
		write_afe(ic_num, 5, 0xD4); //lsb of Pixel count where sh2 goes low to high (0x0AD4=2772)
		write_afe(ic_num, 8, 0x0A); //msb of Pixel count where sh3 goes low to high
		write_afe(ic_num, 9, 0xD4); //lsb of Pixel count where sh3 goes low to high (0x0AD4=2772)
		write_afe(ic_num, 12, 0xFF); //MSBof Pixel count where sh4 goes low to high
		write_afe(ic_num, 16, 0x0A); //msb of the signal sh5 low to high pixel (0x0AD4=2772)
		write_afe(ic_num, 17, 0xD4); //lsb of the signal sh5 low to high pixel
		write_afe(ic_num, 31, 0x08); // to page 8

		// page 8
		if(!ic_num) //master afe, afe-0
		{
			write_afe(ic_num, 0, 0x55); //activate phia1& phia2
			write_afe(ic_num, 1, 0x55); //activate phib1& phib2
			write_afe(ic_num, 2, 0x00); //activate phic1& phic2
			write_afe(ic_num, 3, 0x55); //activate rs &cp
			write_afe(ic_num, 4, 0x55); //activate sh1&sh2
			write_afe(ic_num, 5, 0x00); //disable sh3 & sh4
			write_afe(ic_num, 6, 0x40); //activate sh5
			write_afe(ic_num, 8, 0x60); //lvds out enable, cmos clkout disable
		}
		else //slave afe, afe-1
		{
			//since everything is driven by master no need to generate this outputs
			write_afe(ic_num, 0, 0x00); //disable phia1& phia2
			write_afe(ic_num, 1, 0x00); //disable phib1& phib2
			write_afe(ic_num, 2, 0x00); //disable phic1& phic2
			write_afe(ic_num, 3, 0x00); //disable rs &cp
			write_afe(ic_num, 4, 0x00); //disable sh1&sh2
			write_afe(ic_num, 5, 0x00); //disable sh3 & sh4
			write_afe(ic_num, 6, 0x00); //disable sh5
			write_afe(ic_num, 8, 0x60); //lvds out enable, cmos clkout disable
		}
		write_afe(ic_num, 31, 0x00); // to page 0

		// page 0
		if(!ic_num)//master afe, afe-1
			write_afe(ic_num, 0, 0x03); // Master mode
		else
			write_afe(ic_num, 0, 0x01); // non master
		//end of babishov
	return;
}

int init_vdma(void)
{
	int Status;
	int i;

	XAxiVdma_Config *vdma_config;
	XAxiVdma_DmaSetup vdma_write_config;

	vdma_config = XAxiVdma_LookupConfig(XPAR_AXI_VDMA_0_DEVICE_ID);
	if (!vdma_config) {
		xil_printf("No video DMA found for ID %d\r\n",XPAR_AXI_VDMA_0_DEVICE_ID );
		return XST_FAILURE;
	}

	Status = XAxiVdma_CfgInitialize(&vdma_afe1, vdma_config, vdma_config->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Configuration Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}

	vdma_config = XAxiVdma_LookupConfig(XPAR_AXI_VDMA_1_DEVICE_ID);
	if (!vdma_config) {
		xil_printf("No video DMA found for ID %d\r\n",XPAR_AXI_VDMA_1_DEVICE_ID );
		return XST_FAILURE;
	}

	Status = XAxiVdma_CfgInitialize(&vdma_afe2, vdma_config, vdma_config->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Configuration Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}

	vdma_write_config.VertSizeInput = 1;
	vdma_write_config.HoriSizeInput = 8100 * 2; //8226 * 2; //2byte data width
	vdma_write_config.Stride = 8100 * 2; //8226 * 2; //2byte data width
	vdma_write_config.FrameDelay = 0;  /* This example does not test frame delay */
	vdma_write_config.EnableCircularBuf = 1;
	vdma_write_config.EnableSync = 1;  /*  Gen-Lock */
	vdma_write_config.PointNum = 0;
	vdma_write_config.EnableFrameCounter = 0; /* Endless transfers */
	vdma_write_config.FixedFrameStoreAddr = 0; /* We are not doing parking */

	Status = XAxiVdma_DmaConfig(&vdma_afe1, XAXIVDMA_WRITE, &vdma_write_config);
	if (Status != XST_SUCCESS) {
		xil_printf(
			"Write channel config failed %d\r\n", Status);

		return Status;
	}

	Status = XAxiVdma_DmaConfig(&vdma_afe2, XAXIVDMA_WRITE, &vdma_write_config);
	if (Status != XST_SUCCESS) {
		xil_printf(
			"Write channel config failed %d\r\n", Status);

		return Status;
	}

    vdma_write_config.FrameStoreStartAddr[0] = MEM_BASE_ADDR;
	for(i = 1; i < vdma_afe1.MaxNumFrames; i++)
	{
		vdma_write_config.FrameStoreStartAddr[i] = vdma_write_config.FrameStoreStartAddr[i-1] + 0x8000;
	}

    //register frame buffer addresses
	Status = XAxiVdma_DmaSetBufferAddr(&vdma_afe1, XAXIVDMA_WRITE, vdma_write_config.FrameStoreStartAddr);
	if (Status != XST_SUCCESS) {
		xil_printf("Write channel set buffer address failed %d\r\n", Status);
		return XST_FAILURE;
	}

	vdma_write_config.FrameStoreStartAddr[0] = MEM_BASE_ADDR2;
	for(i = 1; i < vdma_afe2.MaxNumFrames; i++)
	{
		vdma_write_config.FrameStoreStartAddr[i] = vdma_write_config.FrameStoreStartAddr[i-1] + 0x8000;
	}

	//register frame buffer addresses
	Status = XAxiVdma_DmaSetBufferAddr(&vdma_afe2, XAXIVDMA_WRITE, vdma_write_config.FrameStoreStartAddr);
	if (Status != XST_SUCCESS) {
		xil_printf("Write channel set buffer address failed %d\r\n", Status);
		return XST_FAILURE;
	}

	Status = XAxiVdma_DmaStart(&vdma_afe1, XAXIVDMA_WRITE);
	if (Status != XST_SUCCESS) {
		if(Status == XST_VDMA_MISMATCH_ERROR)
			xil_printf("DMA Mismatch Error\r\n");
		return XST_FAILURE;
	}

	Status = XAxiVdma_DmaStart(&vdma_afe2, XAXIVDMA_WRITE);
	if (Status != XST_SUCCESS) {
		if(Status == XST_VDMA_MISMATCH_ERROR)
			xil_printf("DMA Mismatch Error\r\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

u32 afe1_last_write;

static void WriteCallBack(void *CallbackRef, u32 Mask)
{
	/* User can add his code in this call back function */
	XAxiVdma * vdma_ptr = (XAxiVdma *)CallbackRef;

	afe1_last_write = XAxiVdma_CurrFrameStore(vdma_ptr, XAXIVDMA_WRITE);
	if(afe1_last_write == 0)
	{
		memcpy(fb1, (void *)MEM_BASE_ADDR, sizeof(fb1));
	}

	return;
}

static void WriteErrorCallBack(void *CallbackRef, u32 Mask)
{

	/* User can add his code in this call back function */
	;

}

void timer_callback(XScuTimer * TimerInstance)
{
	static int DetectEthLinkStatus = 0;
	/* we need to call tcp_fasttmr & tcp_slowtmr at intervals specified
	 * by lwIP. It is not important that the timing is absoluetly accurate.
	 */
	DetectEthLinkStatus++;

#ifndef USE_SOFTETH_ON_ZYNQ
	ResetRxCntr++;
#endif

	/* For providing an SW alternative for the SI #692601. Under heavy
	 * Rx traffic if at some point the Rx path becomes unresponsive, the
	 * following API call will ensures a SW reset of the Rx path. The
	 * API xemacpsif_resetrx_on_no_rxdata is called every 100 milliseconds.
	 * This ensures that if the above HW bug is hit, in the worst case,
	 * the Rx path cannot become unresponsive for more than 100
	 * milliseconds.
	 */
#ifndef USE_SOFTETH_ON_ZYNQ
	if (ResetRxCntr >= RESET_RX_CNTR_LIMIT) {
		xemacpsif_resetrx_on_no_rxdata(netif);
		ResetRxCntr = 0;
	}
#endif
	/* For detecting Ethernet phy link status periodically */
	if (DetectEthLinkStatus == ETH_LINK_DETECT_INTERVAL) {
		eth_link_detect(netif);
		DetectEthLinkStatus = 0;
	}

	XScuTimer_ClearInterruptStatus(TimerInstance);
}

static int SetupIntrSystem(void)
{
	int Status;
    XScuGic *IntcInstancePtr = &Intc;    /* Instance of the Interrupt Controller */
    XScuGic_Config *IntcConfig;

	IntcConfig = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
	if (NULL == IntcConfig)
		return XST_FAILURE;

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	XScuGic_CPUWriteReg(IntcInstancePtr, XSCUGIC_EOI_OFFSET, XPAR_FABRIC_AXIVDMA_0_VEC_ID); //to enable interrupts after soft reset

    XScuGic_SetPriorityTriggerType(IntcInstancePtr, XPAR_FABRIC_AXIVDMA_0_VEC_ID, 0xA0, 0x3);
    Status = XScuGic_Connect(IntcInstancePtr, XPAR_FABRIC_AXIVDMA_0_VEC_ID,
                    (Xil_InterruptHandler)XAxiVdma_WriteIntrHandler, &vdma_afe1);

    XScuGic_Enable(IntcInstancePtr, XPAR_FABRIC_AXIVDMA_0_VEC_ID);

	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler, (void *)IntcInstancePtr);

	XScuGic_Connect(IntcInstancePtr, XPAR_SCUTIMER_INTR, (Xil_ExceptionHandler)timer_callback,
					(void *)&TimerInstance);

	XScuGic_Enable(IntcInstancePtr, XPAR_SCUTIMER_INTR);
	Xil_ExceptionEnable();

	/* Register call-back functions
	 */
	//XAxiVdma_SetCallBack(&vdma_afe1, XAXIVDMA_HANDLER_GENERAL, WriteCallBack, (void *)&vdma_afe1, XAXIVDMA_WRITE);

	//XAxiVdma_SetCallBack(&vdma_afe1, XAXIVDMA_HANDLER_ERROR, WriteErrorCallBack, (void *)&vdma_afe1, XAXIVDMA_WRITE);

	XAxiVdma_IntrEnable(&vdma_afe1, XAXIVDMA_IXR_ALL_MASK, XAXIVDMA_WRITE);
	return XST_SUCCESS;
}

void platform_setup_timer(void)
{
	int Status = XST_SUCCESS;
	XScuTimer_Config *ConfigPtr;
	int TimerLoadValue = 0;

	ConfigPtr = XScuTimer_LookupConfig(XPAR_SCUTIMER_DEVICE_ID);
	Status = XScuTimer_CfgInitialize(&TimerInstance, ConfigPtr,
			ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS) {

		xil_printf("In %s: Scutimer Cfg initialization failed...\r\n",
		__func__);
		return;
	}

	Status = XScuTimer_SelfTest(&TimerInstance);
	if (Status != XST_SUCCESS) {
		xil_printf("In %s: Scutimer Self test failed...\r\n",
		__func__);
		return;

	}

	XScuTimer_EnableAutoReload(&TimerInstance);
	/*
	 * Set for 250 milli seconds timeout.
	 */
	TimerLoadValue = XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 8;

	XScuTimer_LoadTimer(&TimerInstance, TimerLoadValue);
	return;
}

static void initUDPserver(void)
{
	ip_addr_t ipaddr, netmask, gw;
	ip_addr_t ip_remote;
	err_t error;
	u16_t Port;
	int buflen;

	/* the mac address of the board. this should be unique per board */
	unsigned char mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

	netif = &server_netif;

	platform_setup_timer();

	/* initialize IP addresses to be used */
	IP4_ADDR(&ipaddr,  192, 168,   1, 202);
	IP4_ADDR(&netmask, 255, 255, 255,  0);
	IP4_ADDR(&gw,      192, 168,   1,  1);

	lwip_init();

	/* Add network interface to the netif_list, and set it as default */
	if (!xemac_add(netif, &ipaddr, &netmask, &gw, mac_ethernet_address, XPAR_XEMACPS_0_BASEADDR))
	{
		xil_printf("Error adding N/W interface\n\r");
		return;
	}

	netif_set_default(netif);

	/* now enable interrupts */
	Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);
	XScuTimer_EnableInterrupt(&TimerInstance);
	XScuTimer_Start(&TimerInstance);

	/* specify that the network if is up */
	netif_set_up(netif);

	sleep(1);//sleep 1sec to send arp message
	//udp sender
	Port = UDP_SEND_PORT;
	IP4_ADDR(&ip_remote, 192, 168, 1, 199); // pc

	udp_1 = udp_new();

	error = udp_bind(udp_1, IP_ADDR_ANY, Port);
	if (error != 0)
		xil_printf("Failed %d\r\n", error);
	else if (error == 0)
		xil_printf("Success in UDP binding \r\n");

	error = udp_connect(udp_1, &ip_remote, Port);
	if (error != 0)
		xil_printf("Failed %d\r\n", error);
	else if (error == 0)
		xil_printf("Success in UDP connect \r\n");

	buflen = UDP_PAYLOAD_LENGTH;

	p = pbuf_alloc(PBUF_TRANSPORT, buflen, PBUF_POOL);

	if (!p)
	{
		xil_printf("error allocating pbuf \r\n");
		return;
	}
}

/* entry point of the application */
int main(void)
{
	XGpio_Config *gpio_lm_config;
	int status;
	unsigned temp;
	int led_status = 0;
	u8 counter;

	Xil_DCacheDisable(); //use this otherwise cpu uses cached old data

	/* set heartbeat on */
	init_heartbeat();
	XGpio_DiscreteWrite(&gpio_led0, 1, 5); //green

	/* initialize gpio driver */
	gpio_lm_config = XGpio_LookupConfig(XPAR_GPIO_0_DEVICE_ID);
	status = XGpio_CfgInitialize(&gpio_lm, gpio_lm_config, gpio_lm_config->BaseAddress);
	if(status != XST_SUCCESS)
	{
		return 1; //something went wrong
	}

	temp = XGpio_GetDataDirection(&gpio_lm, 1);
	temp &= 0xFFFFFC00; //set the lowest 10 bit 0
	XGpio_SetDataDirection(&gpio_lm, 1, temp);

	XGpio_DiscreteWrite(&gpio_lm, 1, 0x00000000); //set all outputs to zero
	usleep(5000); //sleep 5ms
	XGpio_DiscreteSet(&gpio_lm, 1, 0x00000022); //set rst to high
	usleep(5000); //sleep 5ms

	/* initialize spi driver */
	init_spi();

	afe_config(0);
	afe_config(1);
	XGpio_DiscreteWrite(&gpio_led0, 1, 3); //blue

	usleep(10000); //wait to let afe start
	status = init_vdma();
	if(status != XST_SUCCESS)
		return 1; //something went wrong

	SetupIntrSystem();

	initUDPserver();
	XGpio_DiscreteWrite(&gpio_led0, 1, 6); //red


	counter = 0;
	//uint regVal1, regVal2;
	while(1)
	{
		if(led_status == 6)
			led_status = 5;
		else if(led_status == 5)
			led_status = 3;
		else
			led_status = 6;
		XGpio_DiscreteWrite(&gpio_led0, 1, led_status);

		xemacif_input(netif);

/*
		regVal1 = Xil_In32(vdma_afe1.BaseAddr + 0x30);
		regVal2 = Xil_In32(vdma_afe2.BaseAddr + 0x30);

		Xil_Out32(vdma_afe1.BaseAddr + 0x34, 0xFFFFFFFF);
		regVal1 = Xil_In32(vdma_afe1.BaseAddr + 0x34);

		Xil_Out32(vdma_afe2.BaseAddr + 0x34, 0xFFFFFFFF);
		regVal2 = Xil_In32(vdma_afe2.BaseAddr + 0x34);*/

		int i;

		counter = 0;
		unsigned short *payload_u8_ptr;
		unsigned short *odd_u8_ptr;
		unsigned short *even_u8_ptr;

		payload_u8_ptr = p->payload;
		odd_u8_ptr = (unsigned short *)MEM_BASE_ADDR;
		even_u8_ptr = (unsigned short *)MEM_BASE_ADDR2;


		for(counter = 0; counter < 54; counter++)
		{
			memcpy(p->payload, &counter, sizeof(u8));
			//memcpy(p->payload+1, (int *)addr_count, MEM_BASE_ADDR + 1000);
			//addr_count += 1000;
			for(i = 0; i < 50; i++)
			{
				payload_u8_ptr[1+(i*6)] = *odd_u8_ptr;
				odd_u8_ptr++;
				payload_u8_ptr[1+(i*6)+1] = *odd_u8_ptr;
				odd_u8_ptr++;
				payload_u8_ptr[1+(i*6)+2] = *odd_u8_ptr;
				odd_u8_ptr++;
				payload_u8_ptr[1+(i*6)+3] = *odd_u8_ptr;
				odd_u8_ptr++;
				payload_u8_ptr[1+(i*6)+4] = *odd_u8_ptr;
				odd_u8_ptr++;
				payload_u8_ptr[1+(i*6)+5] = *odd_u8_ptr;
				odd_u8_ptr++;

				payload_u8_ptr[1+(i*6) +6] = *even_u8_ptr;
				even_u8_ptr++;
				payload_u8_ptr[1+(i*6)+7] = *even_u8_ptr;
				even_u8_ptr++;
				payload_u8_ptr[1+(i*6)+8] = *even_u8_ptr;
				even_u8_ptr++;
				payload_u8_ptr[1+(i*6)+9] = *even_u8_ptr;
				even_u8_ptr++;
				payload_u8_ptr[1+(i*6)+10] = *even_u8_ptr;
				even_u8_ptr++;
				payload_u8_ptr[1+(i*6)+11] = *even_u8_ptr;
				even_u8_ptr++;

				//memcpy(payload_u8_ptr+1+(i*6), (unsigned char *)(MEM_BASE_ADDR + (counter*300) + (i*6)), 6);
				//memcpy(payload_u8_ptr+1+(i*6)+6, (unsigned char *)(MEM_BASE_ADDR2 + (counter*300) + (i*6)), 6);
			}
			udp_send(udp_1, p);
		}
		usleep(1000 * 20);
		//sleep(1);
	}


	return 0;
}
