//Power on
// First two commands cycle soft reset
LM98620_SPI_Write(0x06, 0x01);//Configuration settings 6, Soft Reset, FSM Reset, programmable registers are not disturbed. 
//Wait 20ms
LM98620_SPI_Write(0x06, 0x00);//Configuration settings 6, Soft Reset, REG Reset, reset all FSM, except micro-wire interface, and programmable registers
//Wait 20ms
// Enable Serial Interface
LM98620_SPI_Write(0x01, 0x08);//Configuration settings 1, Interface Configuration (pg48)
// 0xCA resolves to 1100 1010
// bit 1: Enable Sample Timing Pulses to TESTO
// bit 3: Select 6 channel input mode
// bit 6: Enable Passive Input Bias - see page 48
// bit 7: Enable Active Input Bias 
LM98620_SPI_Write(0x00, 0xCA);//Configuration settings 0, Main Configuration (pg48)
//Wait 20ms
// 0x0E resolves to 1110
// bit 1 same
// bit 2: disable VCLP internal buffer
// bit 3: Select 6 ch input mode
// then disable input biases
LM98620_SPI_Write(0x00, 0x0e);//Configuration settings 0, Main Configuration (pg48)

// CLP_CONFG Sample timing control set
// 0x9D resolves to 1001 1101
// bit 0 clamp gated by internal sampling pulse
// bit 1 sampling mode control, see page 36 for details
// bit 3:2 = 11 default for 6ch mode
// bit 4 Active high SHP/SHD input polarity
// bit 7 sampling mode control, see page 36 for details

LM98620_SPI_Write(0x02, 0x9d);//Configuration settings 2, Clamp Control (pg49)
LM98620_SPI_Write(0x02, 0x9d);//Configuration settings 2, Clamp Control(pg49)

// CDSG_CONFIG
// 0x80 to 1000 0000
// Positive input signal polarity (sample and hold mode only)
// 1x gain for all ch
LM98620_SPI_Write(0x03, 0x80);;//Configuration settings 3, FDAC Range, CDS Gain Selection 

// Main config 4 register
// 0x03 to 0000 0011
// bit 0: TXCLK and ADCCLK are 2x MCLK
// bit 1: SH3 mode is disabled
LM98620_SPI_Write(0x04, 0x03);;//Configuration settings 4, Main Configuration 4 (pg50);

// Main config 5 reg
// Only bit 7 is relevant
// bit 7 high: Sample clocks from internal DLL
LM98620_SPI_Write(0x05, 0xf7);//Configuration settings 5, Main Configuration 5 (pg50);

// GA_R1 reg
// Red ch 1 PGA gain
// 1x

// Following regs up to 0x0C are all set for PGA, GAIN, CDAC, FDAC
LM98620_SPI_Write(0x08, 0x00);//GA_R1, RED CHANNEL PGA GAIN, CDAC and FDAC OFFSETS (pg50);

LM98620_SPI_Write(0x09, 0x00);//C_OFFS_R1, [4:0] = Red Channel 1 Offset DAC Code (pg50);
LM98620_SPI_Write(0x0c, 0x00);//GA_R2, [7:0] = Red Channel 2 PGA Gain (pg50);
LM98620_SPI_Write(0x0d, 0x00);//C_OFFS_R2, Red Channel 2 Offset DAC Code
LM98620_SPI_Write(0x10, 0x00);//0x10 to 0x13OS_G1 (Green Even); Channel Gain & Offset Registers,[7:0] = Green Channel 1 PGA Gain
LM98620_SPI_Write(0x11, 0x00);//0x10 to 0x13OS_G1 (Green Even); Channel Gain & Offset Registers, [4:0] = Green Channel 1 Offset DAC Code
LM98620_SPI_Write(0x14, 0x00);//0x14 to 0x17OS_G2 (Green Odd); Channel Gain & Offset Registers, [7:0] = Green Channel 2 PGA Gain
LM98620_SPI_Write(0x15, 0x00);//0x14 to 0x17OS_G2 (Green Odd); Channel Gain & Offset Registers, [4:0] = Green Channel 2 Offset DAC Code
LM98620_SPI_Write(0x18, 0x00);//0x18 to 0x1B OS_B1 (Blue Even); Channel Gain & Offset Registers, [7:0] =Blue Channel 1 PGA Gain
LM98620_SPI_Write(0x19, 0x00);//0x18 to 0x1B OS_B1 (Blue Even); Channel Gain & Offset Registers, [4:0] = Blue Channel 1 Offset DAC Code
LM98620_SPI_Write(0x1c, 0x00);//0x1C to 0x1F OS_B2 (Blue Odd); Channel Gain & Offset Registers, [7:0] = Blue Channel 2 PGA Gain
LM98620_SPI_Write(0x1d, 0x00);//0x1C to 0x1F OS_B2 (Blue Odd); Channel Gain & Offset Registers, [4:0] = Blue Channel 2 Offset DAC Code

// BLKCLP_CTL0 reg
// bit [7:6] = 00 infinite # of lines
// bit 4 = divide by 4/3
// bit 3 = disable auto blkclp pulse gen
// bit 2 = disable auto black loop
// bit 1 = disable high speed mode
// bit 0 = auto black loop mode, update cdac and fdac offset corrections
LM98620_SPI_Write(0x23, 0x00);//Black Level Loop Control, Black Level Loop Control (pg52);

// BLKCLP_CTRL1
// Digital black level clamp control
// bits [7:3] = pixel averaging, 4 pixels
// bits [2:0] = offset integration, divide-by-2
LM98620_SPI_Write(0x24, 0x00);//Black Level Loop Settings, Digital Black Level Clamp Control (pg53);

// AGC_CONFIG
// White level gain cal regs
// 0x10 resolves to  0001 0000
// bit 7: set binary search
// bit 6: dont use BLK_AVG
// bit 5: CLPIN initiates white loop
// bit 4: Disable use of AGC_ON to start white cal loop
// bit 0: disable white level loop
LM98620_SPI_Write(0x28, 0x10);//White Level Loop Control, WHITE LEVEL GAIN CALIBRATION AGC_CONFG (pg54);

// regs 0x2a and 0x2b
// Set starting pixel for peak detect at 0 pixels
LM98620_SPI_Write(0x2a, 0x00);//REG_PK_DET_ST_MSB, Starting pixel for peak detection (pg54);
LM98620_SPI_Write(0x2b, 0x00);//0x2B REG_PK_DET_ST_LSB, (empty); ,(pg54);

// Peak detect duration settings
LM98620_SPI_Write(0x2c, 0x00);//PK_DET_WID_MSB, Duration of peak detection after PK_DET_ST. 16 bit value (pg54);
LM98620_SPI_Write(0x2d, 0x00);//PK_DET_WID_LSB, (empty); ,(pg54);

// DLL sample position
// bits [4:0]
// Delay 19/32 of Tpixel from pixel clock
LM98620_SPI_Write(0x36, 0x13);//0x34 to 0x37 TBD, DLL Sample Position, Must be kept with Power-on-default values.

// DLL sample width
// bits [7:5]
// 10/32 of Tpixel
LM98620_SPI_Write(0x37, 0x80);//TBD, DLL Sample Width [7:5] = Sample Pulse Width (pg55);

// Disable test pattern mode
LM98620_SPI_Write(0x38, 0x00);//Test Pattern Mode (pg56);
LM98620_SPI_Write(0x39, 0x00);//Test Pattern Settings 1, TESTPLVL_MSB, [7:0] = 8 MSb of fixed output code (TESTPLVL)
LM98620_SPI_Write(0x3a, 0x00);//Test Pattern Settings 2, TESTPLVL_LSB, [7:6] = 2 LSb of fixed output code (TESTPLVL)
LM98620_SPI_Write(0x3b, 0x00);//PATW, [7:0] = Gradation Pattern Pitch (0 to 255 lines);
LM98620_SPI_Write(0x3c, 0x00);//PATS, [7:0] = Gradation Pattern Increment Step (0 to 255);
LM98620_SPI_Write(0x3d, 0x00);//LINE_INTVL [3:0] = Test Pattern Output Color Delay, Red to Green, Green to Blue(0 to 15 line delay);


LM98620_SPI_Write(0x3f, 0x80);//Reserved, Select Register Pages, 0x00: Page 0, 0x80: Page 128 (DLL features);

// Start setting DLL
// OS_R1 Sample Falling Edge Position
// 10/32 of TPixel
// Same for OS_R2, OS_G1, OS_G2, OS_B1, OS_B2
LM98620_SPI_Write(0x00, 0x13);//Configuration settings 0, ANLG_CONFG Main Configuration
LM98620_SPI_Write(0x01, 0x13);//Configuration settings 1, INTF_CONFG Interface Configuration
LM98620_SPI_Write(0x02, 0x13);//Configuration settings 2, CLP_CONFG Sample Timing Control

LM98620_SPI_Write(0x03, 0x13);//Configuration settings 3, CDSG_CONFIG, CDS / SH Gain Enable FDAC Range Select(pg 49);
LM98620_SPI_Write(0x04, 0x13);//Configuration settings 4, (pg50);
LM98620_SPI_Write(0x05, 0x13);//Configuration settings 5, (pg50);

// Revert page settings to normal
LM98620_SPI_Write(0x3f, 0x00);// PAGE_SEL for Page Control, Select Register Pages, 0x00: Page 0, 0x80: Page 128 (DLL features)(pg56)