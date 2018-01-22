/*
  low level driver for the beken radio on SPI
*/

#include "driver_bk2425.h"
#include <utility>

#pragma GCC optimize("O0")

const uint8_t Bank1_RegTable[ITX_MAX][IREG_MAX][5]={
	// (TX_SPEED == 250u)
	{
		{ BK2425_R1_4,  0xf9,0x96,0x8a,0xdb }, // 0xDB8A96f9ul, // REG4 250kbps
		{ BK2425_R1_5,  0x24,0x06,0x0f,0xb6 }, // 0xB60F0624ul, // REG5 250kbps
		PLL_SPEED,                                              // REG12
		{ BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // REG13
		{ BK2425_R1_4,  0xff,0x96,0x8a,0xdb }, // 0xDB8A96f9ul, // REG4 250kbps
	},
	// (TX_SPEED == 1000u)
	{
		{ BK2425_R1_4,  0xf9,0x96,0x82,0x1b }, // 0x1B8296f9ul, // REG4 1Mbps
		{ BK2425_R1_5,  0x24,0x06,0x0f,0xa6 }, // 0xA60F0624ul, // REG5 1Mbps
		PLL_SPEED,                                              // REG12
		{ BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // REG13
		{ BK2425_R1_4,  0xff,0x96,0x82,0x1b }, // 0x1B8296f9ul, // REG4 1Mbps
	},
	// (TX_SPEED == 2000u)
	{
		{ BK2425_R1_4,  0xf9,0x96,0x82,0xdb }, // 0xdb8296f9ul, // REG4 2Mbps
		{ BK2425_R1_5,  0x24,0x06,0x0f,0xb6 }, // 0xb60f0624ul, // REG5 2Mbps
		PLL_SPEED,                                              // REG12
		{ BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // REG13
		{ BK2425_R1_4,  0xff,0x96,0x82,0xdb }, // 0xdb8296f9ul, // REG4 2Mbps
	}
};

static const uint8_t Bank0_Reg6[ITX_MAX][2] = {
	{BK_RF_SETUP,   0x27}, //  250kbps (6) 0x27=250kbps
	{BK_RF_SETUP,   0x07}, // 1000kbps (6) 0x07=1Mbps, high gain, high txpower
	{BK_RF_SETUP,   0x2F}, // 2000kbps (6) 0x2F=2Mbps, high gain, high txpower
};

static const uint8_t Bank1_Reg14[]=
{
0x41,0x20,0x08,0x04,0x81,0x20,0xcf,0xF7,0xfe,0xff,0xff
};

// Bank0 register initialization value
static const uint8_t Bank0_Reg[][2]={
    
    {BK_CONFIG,     BK_CONFIG_EN_CRC | BK_CONFIG_CRCO | BK_CONFIG_PWR_UP | BK_CONFIG_PRIM_RX }, // (0) 0x0F=Rx, PowerUp, crc16, all interrupts enabled
    {BK_EN_AA,      0x00}, // (1) 0x00=No auto acknowledge packets on all 6 data pipes (0..5)
    {BK_EN_RXADDR,  0x03}, // (2) 0x01=1 or 2 out of 6 data pipes enabled (pairing heartbeat and my tx)
    {BK_SETUP_AW,   0x03}, // (3) 0x03=5 byte address width
    {BK_SETUP_RETR, 0x00}, // (4) 0x00=No retransmissions
    {BK_RF_CH,      0x17}, // (5) 0x17=2423Mhz default frequency
    
    // Comment in Beken code says that 0x0F or 0x2F=2Mbps; 0x07=1Mbps; 0x27=250Kbps
    #if (TX_SPEED == 2000)
        {BK_RF_SETUP,   0x2F},      // (6) 0x2F=2Mbps, high gain, high txpower
    #elif (TX_SPEED == 1000)
        {BK_RF_SETUP,   0x07},      // (6) 0x07=1Mbps, high gain, high txpower
    #elif (TX_SPEED == 250)
        {BK_RF_SETUP,   0x27},       // (6) 0x27=250kbps
        //{BK_RF_SETUP,   0x21},    // (6) 0x27=250kbps, lowest txpower
    #endif
    
    {BK_STATUS,     0x07},          // (7) 7=no effect
    {BK_OBSERVE_TX, 0x00},          // (8) (no effect)
    {BK_CD,         0x00},          // (9) Carrier detect (no effect)
                                    // (10) = 5 byte register
                                    // (11) = 5 byte register
    {BK_RX_ADDR_P2, 0xc3},          // (12) rx address for data pipe 2
    {BK_RX_ADDR_P3, 0xc4},          // (13) rx address for data pipe 3
    {BK_RX_ADDR_P4, 0xc5},          // (14) rx address for data pipe 4
    {BK_RX_ADDR_P5, 0xc6},          // (15) rx address for data pipe 5
                                    // (16) = 5 byte register
    {BK_RX_PW_P0,   0x20},          // (17) size of rx data pipe 0
    {BK_RX_PW_P1,   0x20},          // (18) size of rx data pipe 1
    {BK_RX_PW_P2,   0x20},          // (19) size of rx data pipe 2
    {BK_RX_PW_P3,   0x20},          // (20) size of rx data pipe 3
    {BK_RX_PW_P4,   0x20},          // (21) size of rx data pipe 4
    {BK_RX_PW_P5,   0x20},          // (22) size of rx data pipe 5
    {BK_FIFO_STATUS,0x00},          // (23) fifo status
                                    // (24,25,26,27)
    {BK_DYNPD,      0x3F},          // (28) 0x3f=enable dynamic payload length for all 6 data pipes
    {BK_FEATURE,    BK_FEATURE_EN_DPL | BK_FEATURE_EN_ACK_PAY | BK_FEATURE_EN_DYN_ACK }  // (29) 7=enable ack, no ack, dynamic payload length
};

extern const AP_HAL::HAL& hal;

// constructor
Radio_Beken::Radio_Beken(AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev) :
    dev(std::move(_dev))
{
	// Set the default address
	TX_Address[0] = 0x33;
	TX_Address[1] = RX0_Address[1] = 0x00;
	TX_Address[2] = RX0_Address[2] = 0x59;
	TX_Address[3] = RX0_Address[3] = 0x00;
	TX_Address[4] = RX0_Address[4] = 0x00;
	RX0_Address[0] = 0x31;
	RX1_Address[0] = 0x32;
	RX1_Address[1] = 0x99;
	RX1_Address[2] = 0x59;
	RX1_Address[3] = 0xC6;
	RX1_Address[4] = 0x2D;
}

void Radio_Beken::ReadFifo(uint8_t *dpbuffer, uint8_t len)
{
    (void)dev->read_registers(BK_RD_RX_PLOAD, dpbuffer, len);
}

void Radio_Beken::WriteFifo(const uint8_t *dpbuffer, uint8_t len)
{
    WriteRegisterMulti(BK_W_TX_PAYLOAD_NOACK_CMD, dpbuffer, len);
}

void Radio_Beken::ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length)
{
    (void)dev->read_registers(address, data, length);
}

void Radio_Beken::WriteRegisterMulti(uint8_t address, const uint8_t *data, uint8_t length)
{
    uint8_t buf[length+1];
    buf[0] = address | BK_WRITE_REG;
    memcpy(&buf[1], data, length);
    dev->transfer(buf, length+1, nullptr, 0);
}

uint8_t Radio_Beken::ReadReg(uint8_t reg)
{
    uint8_t ret = 0;
    (void)dev->read_registers(reg | BK_READ_REG, &ret, 1);
    return ret;
}

uint8_t Radio_Beken::ReadStatus(void)
{
    uint8_t ret = 0;
    (void)dev->read_registers(BK_NOP, &ret, 1);
    return ret;
}

uint8_t Radio_Beken::Strobe(uint8_t address)
{
    uint8_t status=0;
    (void)dev->transfer(&address, 1, &status, 1);
    return status;
}

void Radio_Beken::WriteReg(uint8_t address, uint8_t data)
{
    (void)dev->write_register(address | BK_WRITE_REG, data);
}

// Set which register bank we are accessing
void Radio_Beken::SetRBank(uint8_t bank) // 1:Bank1 0:Bank0
{
    uint8_t lastbank = ReadStatus() & BK_STATUS_RBANK; // carl - is this how you read the status register?
    if (!lastbank != !bank)
    {
        uint8_t buf[2];
        buf[0] = BK_ACTIVATE_CMD;
        buf[1] = 0x53;
        dev->transfer(buf, 2, nullptr, 0);
    }
}

// ----------------------------------------------------------------------------
const uint8_t RegPower[8][2] = {
    { OUTPUT_POWER_REG4_0, OUTPUT_POWER_REG6_0 },
    { OUTPUT_POWER_REG4_1, OUTPUT_POWER_REG6_1 },
    { OUTPUT_POWER_REG4_2, OUTPUT_POWER_REG6_2 },
    { OUTPUT_POWER_REG4_3, OUTPUT_POWER_REG6_3 },
    { OUTPUT_POWER_REG4_4, OUTPUT_POWER_REG6_4 },
    { OUTPUT_POWER_REG4_5, OUTPUT_POWER_REG6_5 },
    { OUTPUT_POWER_REG4_6, OUTPUT_POWER_REG6_6 },
    { OUTPUT_POWER_REG4_7, OUTPUT_POWER_REG6_7 },
};

void Radio_Beken::WriteRegisterMultiBank1(uint8_t address, const uint8_t *data, uint8_t length)
{
    SetRBank(1);
    WriteRegisterMulti(address, data, length);
    SetRBank(0);
}

void Radio_Beken::SetPower(uint8_t power)
{
    if (power > 7) {
        power = 7;
    }
    uint8_t oldready = bkReady;
    bkReady = 0;
    hal.scheduler->delay(100); // delay more than 50ms.
    SetRBank(1);
    {
        const uint8_t* p = &Bank1_RegTable[gTxSpeed][IREG1_4][0];
        uint8_t idx = *p++;
        uint8_t buf[4];
        buf[0] = *p++;
        buf[1] = *p++;
        buf[2] = *p++;
        buf[3] = *p++;
        buf[0] &= ~0x38;
        buf[0] |= (RegPower[power][0] << 3); // Bits 27..29
        WriteRegisterMulti((BK_WRITE_REG|idx), buf, 4);
    }
    hal.scheduler->delay(100); // delay more than 50ms.
    SetRBank(0);
    hal.scheduler->delay(100);

    uint8_t setup = ReadReg(BK_RF_SETUP);
    setup &= ~(3 << 1);
    setup |= (RegPower[power][1] << 1); // Bits 1..2
    WriteReg(BK_RF_SETUP, setup);
    bkReady = oldready;
}

bool Radio_Beken::Reset(void)
{
    hal.scheduler->delay_microseconds(1000);
    return 0;
}

// ----------------------------------------------------------------------------
// Switch to Rx mode
void Radio_Beken::SwitchToRxMode(void)
{
    uint8_t value;
    
    Strobe(BK_FLUSH_RX); // flush Rx
    value = ReadStatus(); // read register STATUS's value
    WriteReg(BK_WRITE_REG|BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag
    
    BEKEN_CE_LOW();
    for (value = 0; value < 40; ++value)
    {	asm volatile("nop"::); }
    value = ReadReg(BK_CONFIG);	// read register CONFIG's value
    value |= BK_CONFIG_PRIM_RX; // set bit 0
    WriteReg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..
    
    BEKEN_CE_HIGH();
    //BEKEN_PA_LOW(); // we dont have a PA on the RX side
}

// ----------------------------------------------------------------------------
// switch to Tx mode
void Radio_Beken::SwitchToTxMode(void)
{
    uint8_t value;
    Strobe(BK_FLUSH_TX); // flush Tx
    
    BEKEN_PA_HIGH();
    BEKEN_CE_LOW();
    for (value = 0; value < 40; ++value)
    {	asm volatile("nop"::); }
    value = ReadReg(BK_CONFIG); // read register CONFIG's value
    value &= ~BK_CONFIG_PRIM_RX; // Clear bit 0 (PTX)
    WriteReg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.
    BEKEN_CE_HIGH();
}

// ----------------------------------------------------------------------------
// switch to Idle mode
void Radio_Beken::SwitchToIdleMode(void)
{
    uint8_t value;
    Strobe(BK_FLUSH_TX); // flush Tx
    
    BEKEN_PA_LOW();
    BEKEN_CE_LOW();
    for (value = 0; value < 40; ++value)
    {   asm volatile("nop"::); }
}

// ----------------------------------------------------------------------------
// Switch to Sleep mode
void Radio_Beken::SwitchToSleepMode(void)
{
    uint8_t value;
    
    Strobe(BK_FLUSH_RX); // flush Rx
    Strobe(BK_FLUSH_TX); // flush Tx
    value = ReadStatus(); // read register STATUS's value
    WriteReg(BK_WRITE_REG|BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag
    
    BEKEN_PA_LOW();
    BEKEN_CE_LOW();
    for (value = 0; value < 40; ++value)
    {   asm volatile("nop"::); }
    value = ReadReg(BK_CONFIG);	// read register CONFIG's value
    value |= BK_CONFIG_PRIM_RX; // Receive mode
    value &= ~BK_CONFIG_PWR_UP; // Power down
    WriteReg(BK_WRITE_REG | BK_CONFIG, value); // Clear PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..
    // Stay low
    BEKEN_CE_LOW();
}

// ----------------------------------------------------------------------------
void Radio_Beken::InitBank0Registers(ITX_SPEED spd)
{
    int8_t i;
    
    //********************Write Bank0 register******************
    for (i=20; i >= 0; i--) // From BK_FIFO_STATUS back to beginning of table
    {
        uint8_t idx = Bank0_Reg[i][0];
        uint8_t value = Bank0_Reg[i][1];
        if (idx == BK_RF_SETUP) // Adjust for speed
            value = Bank0_Reg6[spd][1];
        WriteReg((BK_WRITE_REG|idx), value);
    }

    // Set the various 5 byte addresses
    WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P0),RX0_Address,5); // reg 10 - Rx0 addr
    WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P1),RX1_Address,5); // REG 11 - Rx1 addr
    WriteRegisterMulti((BK_WRITE_REG|BK_TX_ADDR),TX_Address,5); // REG 16 - TX addr

    i = ReadReg(BK_FEATURE);
    if (i == 0) // i!=0 showed that chip has been actived.so do not active again.
        WriteReg(BK_ACTIVATE_CMD,0x73);// Active
    for (i = 22; i >= 21; i--)
        WriteReg((BK_WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);
}

// ----------------------------------------------------------------------------
void Radio_Beken::InitBank1Registers(ITX_SPEED spd)
{
    int8_t i;
       
	for (i = IREG1_4; i <= IREG1_13; i++)
    {
        const uint8_t* p = &Bank1_RegTable[spd][i][0];
        uint8_t idx = *p++;
        WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }
    WriteRegisterMulti((BK_WRITE_REG|BK2425_R1_14),&(Bank1_Reg14[0]),11);

//toggle REG4<25,26>
    {
        const uint8_t* p = &Bank1_RegTable[spd][IREG1_4A][0];
        uint8_t idx = *p++;
        WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }
    {
        const uint8_t* p = &Bank1_RegTable[spd][IREG1_4][0];
        uint8_t idx = *p++;
        WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }
}
