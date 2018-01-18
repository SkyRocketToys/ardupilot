//----------------------------------------------------------------------------------
// low level driver for the Beken BK2425 radio on SPI
//----------------------------------------------------------------------------------

#pragma once

#include <AP_HAL/AP_HAL.h>

// Choose between supporting the Nordic nrf24l01+ and the Beken BK2425 radio chips
// The exact revision of the PCB matters, as pins change location
#define PCB_BOARD_PRODUCT 1802
#define PCB_BOARD_REVISION 2

#define RADIO_NRF24 0
#define RADIO_BEKEN 1 // We are using the Beken BK2425 chip
#define SUPPORT_PA 1
#define TX_SPEED 250u // Default transmit speed in kilobits per second.

//----------------------------------------------------------------------------------
// Under ChiBios the knowledge of which pins are which is not in the driver.
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
// Which pins are connected to which devices is stored in:
// AP_HAL_ChibiOS/GPIO.cpp for the LEDs / Chip enable / power amplifier
// This should really be in AP_HAL_ChibiOS/GPIO.h for this subboard
// Reference to GPIO pins for this exact PCB revision
enum {
	HAL_CHIBIOS_GPIO_LEDR = 0,      // Meaning GPIOB, 7 (for 1802-02, 1802-05)
	HAL_CHIBIOS_GPIO_LEDF,          // Meaning GPIOB, 6 (for 1802-02, 1802-05)
    HAL_CHIBIOS_GPIO_RADIO_IRQ,	    // Meaning GPIOD, 2 (for 1802-02) or GPIOB, 0 (for 1802-05)
	HAL_CHIBIOS_GPIO_RADIO_CE,      // Meaning GPIOC, 4 (for 1802-02, 1802-05)
	HAL_CHIBIOS_GPIO_RADIO_PA_CTL	// Meaning GPIOC, 5 (for 1802-02, 1802-05)
};

//----------------------------------------------------------------------------------
// AP_HAL_ChibiOS/SPIDevice.cpp for each chips
// Chip Select (Motion, Radio, Optical flow), SCK, MOSI, MISO
//    SPIDEV_CS_MPU  B12
//    SPIDEV_CS_CYRF A15
//    SPIDEV_CS_FLOW B1

//#if (PCB_BOARD_PRODUCT==1802) && (PCB_REVISION==2) // PCB 1802-02 dated 18 oct 2017
//#define HAL_GPIO_SPI_SCK                 GPIOA,  5
//#define HAL_GPIO_SPI_MOSI                GPIOA,  7
//#define HAL_GPIO_SPI_MISO                GPIOB,  4
//#define HAL_GPIO_RADIO_CS                GPIOA, 15                          Named SPIDEV_CS_CYRF
//#endif

//#if (PCB_BOARD_PRODUCT==1802) && (PCB_REVISION==5) // PCB 1802-05 dated 7 dec 2017
//#define HAL_GPIO_SPI_SCK	               GPIOA,  5
//#define HAL_GPIO_SPI_MOSI	               GPIOA,  7
//#define HAL_GPIO_SPI_MISO                GPIOA,  6 (Different from 1802-02)
//#define HAL_GPIO_RADIO_CS                GPIOA,  4 (Different from 1802-02) Named SPIDEV_CS_CYRF
//#endif

//----------------------------------------------------------------------------------
/** SPI register commands for the BK2425 and nrf24L01+ chips */
typedef enum {
// General commands
	BK_REG_MASK        = 0x1F,  // The range of registers that can be read and written
	BK_READ_REG        = 0x00,  // Define read command to register (0..1F)
	BK_WRITE_REG       = 0x20,  // Define write command to register (0..1F)
#if RADIO_BEKEN
	BK_ACTIVATE_CMD	   = 0x50,
#endif
	BK_R_RX_PL_WID_CMD = 0x60,
	BK_RD_RX_PLOAD     = 0x61,  // Define RX payload register address
	BK_WR_TX_PLOAD     = 0xA0,  // Define TX payload register address
	BK_W_ACK_PAYLOAD_CMD = 0xA8, // (nrf: +pipe 0..7)
	BK_W_TX_PAYLOAD_NOACK_CMD = 0xB0,
	BK_FLUSH_TX        = 0xE1,  // Define flush TX register command
	BK_FLUSH_RX        = 0xE2,  // Define flush RX register command
	BK_REUSE_TX_PL     = 0xE3,  // Define reuse TX payload register command
	BK_NOP             = 0xFF,  // Define No Operation, might be used to read status register

// BK2425 bank 0 register addresses
	BK_CONFIG          = 0x00,  // 'Config' register address
	BK_EN_AA           = 0x01,  // 'Enable Auto Acknowledgment' register address
	BK_EN_RXADDR       = 0x02,  // 'Enabled RX addresses' register address
	BK_SETUP_AW        = 0x03,  // 'Setup address width' register address
	BK_SETUP_RETR      = 0x04,  // 'Setup Auto. Retrans' register address
	BK_RF_CH           = 0x05,  // 'RF channel' register address
	BK_RF_SETUP        = 0x06,  // 'RF setup' register address
	BK_STATUS          = 0x07,  // 'Status' register address
	BK_OBSERVE_TX      = 0x08,  // 'Observe TX' register address (lost packets, retransmitted packets on this frequency)
	BK_CD              = 0x09,  // 'Carrier Detect' register address
	BK_RX_ADDR_P0      = 0x0A,  // 'RX address pipe0' register address (5 bytes)
	BK_RX_ADDR_P1      = 0x0B,  // 'RX address pipe1' register address (5 bytes)
	BK_RX_ADDR_P2      = 0x0C,  // 'RX address pipe2' register address (1 byte)
	BK_RX_ADDR_P3      = 0x0D,  // 'RX address pipe3' register address (1 byte)
	BK_RX_ADDR_P4      = 0x0E,  // 'RX address pipe4' register address (1 byte)
	BK_RX_ADDR_P5      = 0x0F,  // 'RX address pipe5' register address (1 byte)
	BK_TX_ADDR         = 0x10,  // 'TX address' register address (5 bytes)
	BK_RX_PW_P0        = 0x11,  // 'RX payload width, pipe0' register address
	BK_RX_PW_P1        = 0x12,  // 'RX payload width, pipe1' register address
	BK_RX_PW_P2        = 0x13,  // 'RX payload width, pipe2' register address
	BK_RX_PW_P3        = 0x14,  // 'RX payload width, pipe3' register address
	BK_RX_PW_P4        = 0x15,  // 'RX payload width, pipe4' register address
	BK_RX_PW_P5        = 0x16,  // 'RX payload width, pipe5' register address
	BK_FIFO_STATUS     = 0x17,  // 'FIFO Status Register' register address
	BK_DYNPD           = 0x1c,  // 'Enable dynamic payload length' register address
	BK_FEATURE         = 0x1d,  // 'Feature' register address
#if RADIO_BEKEN
	BK_PAYLOAD_WIDTH   = 0x1f,  // 'payload length of 256 bytes modes register address

// BK2425 bank 1 register addresses
	BK2425_R1_4      = 0x04,
	BK2425_R1_5      = 0x05,
	BK2425_R1_WHOAMI = 0x08, // Register to read that contains the chip id
	BK2425_R1_12     = 0x0C, // PLL speed 120 or 130us
	BK2425_R1_13     = 0x0D,
	BK2425_R1_14     = 0x0E,
#endif
} BK_SPI_CMD;

//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------

enum {
	BK_CHIP_ID_BK2425 = 0x63, // The expected value of reading BK2425_R1_WHOAMI
};

// Meanings of the BK_STATUS register
enum {
#if RADIO_BEKEN
	BK_STATUS_RBANK = 0x80, // Register bank 1 is in use
#endif
	BK_STATUS_RX_DR = 0x40, // Data ready
	BK_STATUS_TX_DS = 0x20, // Data sent
	BK_STATUS_MAX_RT = 0x10, // Max retries failed
	BK_STATUS_RX_MASK = 0x0E, // Mask for the receptions bit
	BK_STATUS_RX_EMPTY = 0x0E,
	BK_STATUS_RX_P_5 = 0x0A, // Data pipe 5 has some data ready
	BK_STATUS_RX_P_4 = 0x08, // Data pipe 4 has some data ready
	BK_STATUS_RX_P_3 = 0x06, // Data pipe 3 has some data ready
	BK_STATUS_RX_P_2 = 0x04, // Data pipe 2 has some data ready
	BK_STATUS_RX_P_1 = 0x02, // Data pipe 1 has some data ready
	BK_STATUS_RX_P_0 = 0x00, // Data pipe 0 has some data ready
	BK_STATUS_TX_FULL = 0x01 // Tx buffer full
};

// Meanings of the FIFO_STATUS register
enum {
	BK_FIFO_STATUS_TX_REUSE = 0x40,
	BK_FIFO_STATUS_TX_FULL  = 0x20,
	BK_FIFO_STATUS_TX_EMPTY = 0x10,
	BK_FIFO_STATUS_RX_FULL  = 0x02,
	BK_FIFO_STATUS_RX_EMPTY = 0x01
};

// Meanings of the BK_CONFIG register
enum {
	BK_CONFIG_MASK_RX_DR = 0x40,  // Mask interrupt caused by RX_DR
	BK_CONFIG_MASK_TX_DS = 0x20,  // Mask interrupt caused by TX_DS
	BK_CONFIG_MASK_MAX_RT = 0x10, // Mask interrupt caused by MAX_RT
	BK_CONFIG_EN_CRC = 0x08,      // Enable CRC. Forced high if one of the bits in the EN_AA is high
	BK_CONFIG_CRCO = 0x04,        // CRC encoding scheme (0=8 bits, 1=16 bits)
	BK_CONFIG_PWR_UP = 0x02,      // POWER UP
	BK_CONFIG_PRIM_RX = 0x01,     // Receive/transmit
};

enum {
	BK_FEATURE_EN_DPL = 0x04,     //
	BK_FEATURE_EN_ACK_PAY = 0x02, //
	BK_FEATURE_EN_DYN_ACK = 0x01, //
};

// (Lets make it one radio interface for both projects)

/** The baud rate of the GFSK modulation */
typedef enum ITX_SPEED_e {
	ITX_250,  ///< 250kbps (slowest but furthest range)
	ITX_1000, ///< 1000kbps (balanced)
	ITX_2000, ///< 2000kbps (fastest hence least congested)
	ITX_MAX
} ITX_SPEED;




#if RADIO_BEKEN
#define PLL_SPEED { BK2425_R1_12, 0x00,0x12,0x73,0x05 } // 0x00127305ul, // PLL locking time 130us compatible with nRF24L01;

// In the array Bank1_Reg0_13[],all the register values are the byte reversed!
enum {
	IREG1_4,
	IREG1_5,
	IREG1_12,
	IREG1_13,
	IREG1_4A,
	IREG_MAX
};

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
#endif

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
    {BK_EN_RXADDR,  0x01}, // (2) 0x01=1 or 2 out of 6 data pipes enabled (pairing heartbeat and my tx)
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

#define BK_MAX_PACKET_LEN 32 // max value is 32 bytes
#define BK_RCV_TIMEOUT 30

//----------------------------------------------------------------------------------
// Translate output power into a number
// must match up with the table RegPower[]
#if RADIO_BEKEN
#define OUTPUT_POWER_REG6_0 0 // -25dB
#define OUTPUT_POWER_REG6_1 0 // -18dB
#define OUTPUT_POWER_REG6_2 1 // -18dB
#define OUTPUT_POWER_REG6_3 1 // -12dB
#define OUTPUT_POWER_REG6_4 1 // -12dB
#define OUTPUT_POWER_REG6_5 2 //  -7dB
#define OUTPUT_POWER_REG6_6 3 //  -1dB
#define OUTPUT_POWER_REG6_7 3 //  +4dB
#else // Nrf24 chip
#define OUTPUT_POWER_REG6_0 0 // -18dB
#define OUTPUT_POWER_REG6_1 0 // -18dB
#define OUTPUT_POWER_REG6_2 1 // -12dB
#define OUTPUT_POWER_REG6_3 1 // -12dB
#define OUTPUT_POWER_REG6_4 2 //  -6dB
#define OUTPUT_POWER_REG6_5 2 //  -6dB
#define OUTPUT_POWER_REG6_6 3 //  +0dB
#define OUTPUT_POWER_REG6_7 3 //  +0dB
#endif

// Register 4 in bank 1 only applies to Beken chip
#define OUTPUT_POWER_REG4_0 0 // -25dB
#define OUTPUT_POWER_REG4_1 3 // -18dB
#define OUTPUT_POWER_REG4_2 0 // -18dB
#define OUTPUT_POWER_REG4_3 3 // -12dB
#define OUTPUT_POWER_REG4_4 2 // -12dB
#define OUTPUT_POWER_REG4_5 0 //  -7dB
#define OUTPUT_POWER_REG4_6 0 //  -1dB
#define OUTPUT_POWER_REG4_7 7 //  +4dB

// Generic support
#define TOKENPASTE(x, y) x ## y
#define TOKENPASTE2(x, y) TOKENPASTE(x, y)
// The default register values that are for the default power setting
#define DEFAULT_OUTPUT_REG6 TOKENPASTE2(OUTPUT_POWER_REG6_,DEFAULT_OUTPUT_POWER)
#define DEFAULT_OUTPUT_REG4 TOKENPASTE2(OUTPUT_POWER_REG4_,DEFAULT_OUTPUT_POWER)

// This assumes we are using ChiBios instead of the pixhawk o/s for accessing GPIO
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#error This configuration is not supported.
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
#define BEKEN_SELECT()      (dev->set_chip_select(true))
#define BEKEN_DESELECT()    (dev->set_chip_select(false))
#define BEKEN_CE_HIGH()     (hal.gpio->write(HAL_CHIBIOS_GPIO_RADIO_CE, 1))
#define BEKEN_CE_LOW()      (hal.gpio->write(HAL_CHIBIOS_GPIO_RADIO_CE, 0))
#define BEKEN_PA_HIGH()     (hal.gpio->write(HAL_CHIBIOS_GPIO_RADIO_PA_CTL, 1))
#define BEKEN_PA_LOW()      (hal.gpio->write(HAL_CHIBIOS_GPIO_RADIO_PA_CTL, 0))
#else
#error This configuration is not supported.
#endif
#endif




//----------------------------------------------------------------------------------
// BEKEN driver class
class Radio_Beken {
public:
    Radio_Beken(AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev);

    void ReadFifo(uint8_t *dpbuffer, uint8_t len);
    void WriteFifo(const uint8_t *dpbuffer, uint8_t len);

    void ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length);
    void WriteRegisterMulti(uint8_t address, const uint8_t *data, uint8_t length);
    void WriteRegisterMultiBank1(uint8_t address, const uint8_t *data, uint8_t length);

	uint8_t ReadStatus(void);
    uint8_t ReadReg(uint8_t reg);
    uint8_t Strobe(uint8_t address);
	void SetRBank(uint8_t bank);
    void WriteReg(uint8_t address, uint8_t data);
    void SetPower(uint8_t power);
    bool Reset(void);
	void SwitchToRxMode(void);
	void SwitchToTxMode(void);
	void SwitchToIdleMode(void);
	void SwitchToSleepMode(void);
	
    bool lock_bus(void) {
        return dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER);
    }
    void unlock_bus(void) {
        dev->get_semaphore()->give();
    }
    
    // MOTE: not sure if this is how it should be declared

    uint8_t bkReady; // initialised in AP_Radio_bk2425.h radio_init() at the very end
    
    // Note: this should be moved to be within the class
    #if (TX_SPEED==250)
    ITX_SPEED gTxSpeed = ITX_250;
    #elif (TX_SPEED==100)
    ITX_SPEED gTxSpeed = ITX_1000;
    #elif (TX_SPEED==2000)
    ITX_SPEED gTxSpeed = ITX_2000;
    #endif
    

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
};
