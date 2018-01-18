/*
  low level driver for the beken radio on SPI
*/

#include "driver_bk2425.h"
#include <utility>

#pragma GCC optimize("O0")

extern const AP_HAL::HAL& hal;

// constructor
Radio_Beken::Radio_Beken(AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev) :
    dev(std::move(_dev))
{}

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
#if RADIO_BEKEN
    uint8_t lastbank = ReadStatus() & BK_STATUS_RBANK; // carl - is this how you read the status register?
    if (!lastbank != !bank)
    {
        uint8_t buf[2];
        buf[0] = BK_ACTIVATE_CMD;
        buf[1] = 0x53;
        dev->transfer(buf, 2, nullptr, 0);
    }
#endif
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

#if RADIO_BEKEN
void Radio_Beken::WriteRegisterMultiBank1(uint8_t address, const uint8_t *data, uint8_t length)
{
    SetRBank(1);
    WriteRegisterMulti(address, data, length);
    SetRBank(0);
}
#endif

void Radio_Beken::SetPower(uint8_t power)
{
    if (power > 7) {
        power = 7;
    }
    uint8_t oldready = bkReady;
    bkReady = 0;
#if RADIO_BEKEN
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
#endif

    uint8_t setup = ReadReg(BK_RF_SETUP);
    setup &= ~(3 << 1);
    setup |= (RegPower[power][1] << 1); // Bits 1..2
    WriteReg(BK_RF_SETUP, setup);
    bkReady = oldready;
}

bool Radio_Beken::Reset(void)
{
    // we commented this out
    //Strobe(CC2500_SRES);
    
    hal.scheduler->delay_microseconds(1000);
    // Tridge commented this out
    // CC2500_SetTxRxMode(TXRX_OFF);
    // RX_EN_off;//off tx
    // TX_EN_off;//off rx
    
    // we commented this out
    //return ReadReg(CC2500_0E_FREQ1) == 0xC4; // check if reset
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
