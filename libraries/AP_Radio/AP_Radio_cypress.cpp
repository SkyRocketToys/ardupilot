#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <board_config.h>
#include "AP_Radio_cypress.h"
#include <utility>
#include <stdio.h>

/*
  driver for CYRF6936 radio

  Many thanks to the SuperBitRF project from Paparrazi for their DSM
  configuration code and register defines
   https://github.com/esden/superbitrf-firmware
 */

extern const AP_HAL::HAL& hal;

#define RADIO_DEBUG 1
#if RADIO_DEBUG
#define debug(fmt, args...)   printf(fmt, ##args)
#else
#define debug(fmt, args...)   
#endif

#define LP_FIFO_SIZE  16      // Physical data FIFO lengths in Radio

/* The SPI interface defines */
enum {
    CYRF_CHANNEL    		= 0x00,
    CYRF_TX_LENGTH      	= 0x01,
    CYRF_TX_CTRL	    	= 0x02,
    CYRF_TX_CFG         	= 0x03,
    CYRF_TX_IRQ_STATUS  	= 0x04,
    CYRF_RX_CTRL        	= 0x05,
    CYRF_RX_CFG         	= 0x06,
    CYRF_RX_IRQ_STATUS		= 0x07,
    CYRF_RX_STATUS			= 0x08,
    CYRF_RX_COUNT       	= 0x09,
    CYRF_RX_LENGTH			= 0x0A,
    CYRF_PWR_CTRL       	= 0x0B,
    CYRF_XTAL_CTRL      	= 0x0C,
    CYRF_IO_CFG         	= 0x0D,
    CYRF_GPIO_CTRL      	= 0x0E,
    CYRF_XACT_CFG       	= 0x0F,
    CYRF_FRAMING_CFG    	= 0x10,
    CYRF_DATA32_THOLD   	= 0x11,
    CYRF_DATA64_THOLD   	= 0x12,
    CYRF_RSSI           	= 0x13,
    CYRF_EOP_CTRL       	= 0x14,
    CYRF_CRC_SEED_LSB		= 0x15,
    CYRF_CRC_SEED_MSB		= 0x16,
    CYRF_TX_CRC_LSB			= 0x17,
    CYRF_TX_CRC_MSB			= 0x18,
    CYRF_RX_CRC_LSB			= 0x19,
    CYRF_RX_CRC_MSB			= 0x1A,
    CYRF_TX_OFFSET_LSB  	= 0x1B,
    CYRF_TX_OFFSET_MSB  	= 0x1C,
    CYRF_MODE_OVERRIDE  	= 0x1D,
    CYRF_RX_OVERRIDE    	= 0x1E,
    CYRF_TX_OVERRIDE    	= 0x1F,
    CYRF_TX_BUFFER			= 0x20,
    CYRF_RX_BUFFER			= 0x21,
    CYRF_SOP_CODE			= 0x22,
    CYRF_DATA_CODE			= 0x23,
    CYRF_PREAMBLE			= 0x24,
    CYRF_MFG_ID				= 0x25,
    CYRF_XTAL_CFG			= 0x26,
    CYRF_CLK_OFFSET     	= 0x27,
    CYRF_CLK_EN         	= 0x28,
    CYRF_RX_ABORT       	= 0x29,
    CYRF_AUTO_CAL_TIME  	= 0x32,
    CYRF_AUTO_CAL_OFFSET	= 0x35,
    CYRF_ANALOG_CTRL    	= 0x39,
};
#define CYRF_DIR				(1<<7) /**< Bit for enabling writing */

// CYRF_MODE_OVERRIDE
#define CYRF_RST				(1<<0)

// CYRF_CLK_EN
#define CYRF_RXF				(1<<1)

// CYRF_XACT_CFG
enum {
	CYRF_MODE_SLEEP		= (0x0 <<2),
	CYRF_MODE_IDLE		= (0x1 <<2),
	CYRF_MODE_SYNTH_TX	= (0x2 <<2),
	CYRF_MODE_SYNTH_RX	= (0x3 <<2),
	CYRF_MODE_RX		= (0x4 <<2),
};
#define CYRF_FRC_END			(1<<5)
#define CYRF_ACK_EN				(1<<7)

// CYRF_IO_CFG
#define CYRF_IRQ_GPIO			(1<<0)
#define CYRF_SPI_3PIN			(1<<1)
#define CYRF_PACTL_GPIO			(1<<2)
#define CYRF_PACTL_OD			(1<<3)
#define CYRF_XOUT_OD			(1<<4)
#define CYRF_MISO_OD			(1<<5)
#define CYRF_IRQ_POL			(1<<6)
#define CYRF_IRQ_OD				(1<<7)

// CYRF_FRAMING_CFG
#define CYRF_LEN_EN				(1<<5)
#define CYRF_SOP_LEN			(1<<6)
#define CYRF_SOP_EN				(1<<7)

// CYRF_RX_STATUS
enum {
	CYRF_RX_DATA_MODE_GFSK	= 0x00,
	CYRF_RX_DATA_MODE_8DR	= 0x01,
	CYRF_RX_DATA_MODE_DDR	= 0x10,
	CYRF_RX_DATA_MODE_NV	= 0x11,
};
#define CYRF_RX_CODE			(1<<2)
#define CYRF_BAD_CRC			(1<<3)
#define CYRF_CRC0				(1<<4)
#define CYRF_EOP_ERR			(1<<5)
#define CYRF_PKT_ERR			(1<<6)
#define CYRF_RX_ACK				(1<<7)

// CYRF_TX_IRQ_STATUS
#define CYRF_TXE_IRQ			(1<<0)
#define CYRF_TXC_IRQ			(1<<1)
#define CYRF_TXBERR_IRQ			(1<<2)
#define CYRF_TXB0_IRQ			(1<<3)
#define CYRF_TXB8_IRQ			(1<<4)
#define CYRF_TXB15_IRQ			(1<<5)
#define CYRF_LV_IRQ				(1<<6)
#define CYRF_OS_IRQ				(1<<7)

// CYRF_RX_IRQ_STATUS
#define CYRF_RXE_IRQ			(1<<0)
#define CYRF_RXC_IRQ			(1<<1)
#define CYRF_RXBERR_IRQ			(1<<2)
#define CYRF_RXB1_IRQ			(1<<3)
#define CYRF_RXB8_IRQ			(1<<4)
#define CYRF_RXB16_IRQ			(1<<5)
#define CYRF_SOPDET_IRQ			(1<<6)
#define CYRF_RXOW_IRQ			(1<<7)

// CYRF_TX_CTRL
#define CYRF_TXE_IRQEN			(1<<0)
#define CYRF_TXC_IRQEN			(1<<1)
#define CYRF_TXBERR_IRQEN		(1<<2)
#define CYRF_TXB0_IRQEN			(1<<3)
#define CYRF_TXB8_IRQEN			(1<<4)
#define CYRF_TXB15_IRQEN		(1<<5)
#define CYRF_TX_CLR				(1<<6)
#define CYRF_TX_GO				(1<<7)

// CYRF_RX_CTRL
#define CYRF_RXE_IRQEN			(1<<0)
#define CYRF_RXC_IRQEN			(1<<1)
#define CYRF_RXBERR_IRQEN		(1<<2)
#define CYRF_RXB1_IRQEN			(1<<3)
#define CYRF_RXB8_IRQEN			(1<<4)
#define CYRF_RXB16_IRQEN		(1<<5)
#define CYRF_RSVD				(1<<6)
#define CYRF_RX_GO				(1<<7)

// CYRF_RX_OVERRIDE
#define CYRF_ACE				(1<<1)
#define CYRF_DIS_RXCRC			(1<<2)
#define CYRF_DIS_CRC0			(1<<3)
#define CYRF_FRC_RXDR			(1<<4)
#define CYRF_MAN_RXACK			(1<<5)
#define CYRF_RXTX_DLY			(1<<6)
#define CYRF_ACK_RX				(1<<7)

// CYRF_TX_OVERRIDE
#define CYRF_TX_INV				(1<<0)
#define CYRF_DIS_TXCRC			(1<<2)
#define CYRF_OVRD_ACK			(1<<3)
#define CYRF_MAN_TXACK			(1<<4)
#define CYRF_FRC_PRE			(1<<6)
#define CYRF_ACK_TX				(1<<7)

// CYRF_RX_CFG
#define CYRF_VLD_EN				(1<<0)
#define CYRF_RXOW_EN			(1<<1)
#define CYRF_FAST_TURN_EN		(1<<3)
#define CYRF_HILO				(1<<4)
#define CYRF_ATT				(1<<5)
#define CYRF_LNA				(1<<6)
#define CYRF_AGC_EN				(1<<7)

// CYRF_TX_CFG
enum {
	CYRF_PA_M35		= 0x0,
	CYRF_PA_M30		= 0x1,
	CYRF_PA_M24		= 0x2,
	CYRF_PA_M18		= 0x3,
	CYRF_PA_M13		= 0x4,
	CYRF_PA_M5		= 0x5,
	CYRF_PA_0		= 0x6,
	CYRF_PA_4		= 0x7,
};
enum {
	CYRF_DATA_MODE_GFSK	= (0x0 <<3),
	CYRF_DATA_MODE_8DR	= (0x1 <<3),
	CYRF_DATA_MODE_DDR	= (0x2 <<3),
	CYRF_DATA_MODE_SDR	= (0x3 <<3),
};
#define CYRF_DATA_CODE_LENGTH	(1<<5)


#define FLAG_WRITE      0x80
#define FLAG_AUTO_INC   0x40

// object instance for trampoline
AP_Radio_cypress *AP_Radio_cypress::radio_instance;

/*
  constructor
 */
AP_Radio_cypress::AP_Radio_cypress(AP_Radio &_radio) :
    AP_Radio_backend(_radio)
{
    // link to instance for irq_trampoline
    radio_instance = this;
}

/*
  initialise radio
 */
bool AP_Radio_cypress::init(void)
{
    dev = std::move(hal.spi->get_device("external0m0"));

    return reset();
}

/*
  reset radio
 */
bool AP_Radio_cypress::reset(void)
{
    if (!dev->get_semaphore()->take(0)) {
        return false;
    }

    /*
      use AUX6 for reset line for now, hold it high for 0.5s to reset
      radio then wait 0.5s for it to settle
     */
    stm32_configgpio(GPIO_GPIO5_OUTPUT);
    stm32_gpiowrite(GPIO_GPIO5_OUTPUT, 1);
    hal.scheduler->delay(500);
    stm32_gpiowrite(GPIO_GPIO5_OUTPUT, 0);
    hal.scheduler->delay(500);

    // use AUX5 as radio IRQ pin
    stm32_configgpio(GPIO_GPIO4_INPUT);
    
    radio_init();
    dev->get_semaphore()->give();
    return true;
}

/*
  send len bytes as a single packet
 */
bool AP_Radio_cypress::send(const uint8_t *pkt, uint16_t len)
{
    return streaming_transmit(pkt, len);
}

/*
  receive up to maxlen bytes as one packet, return the packet length
 */
uint8_t AP_Radio_cypress::recv(uint8_t *pkt, uint16_t maxlen)
{
    if (!dev->get_semaphore()->take(0)) {
        return false;
    }
    uint8_t len = streaming_receive(pkt, maxlen);
    dev->get_semaphore()->give();
    return len;
}

/*
  go to next channel
 */
void AP_Radio_cypress::next_channel(void)
{
    dsm_set_next_channel();
}


/* The PN codes */
const uint8_t AP_Radio_cypress::pn_codes[5][9][8] = {
{ /* Row 0 */
  /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
  /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
  /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
  /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
  /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
  /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
  /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
  /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
  /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
},
{ /* Row 1 */
  /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
  /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
  /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
  /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
  /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
  /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
  /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
  /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
  /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
},
{ /* Row 2 */
  /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
  /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
  /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
  /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
  /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
  /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
  /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
  /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
  /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
},
{ /* Row 3 */
  /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
  /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
  /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
  /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
  /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
  /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
  /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
  /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
  /* Col 8 */ {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40}
},
{ /* Row 4 */
  /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
  /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
  /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
  /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
  /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
  /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
  /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
  /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
  /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
},
};
const uint8_t AP_Radio_cypress::pn_bind[] = { 0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84 };

/*The CYRF initial config, binding config and transfer config */
const AP_Radio_cypress::config AP_Radio_cypress::cyrf_config[] = {
		{CYRF_MODE_OVERRIDE, CYRF_RST},											// Reset the device
		{CYRF_CLK_EN, CYRF_RXF},												// Enable the clock
		{CYRF_AUTO_CAL_TIME, 0x3C},												// From manual, needed for initialization
		{CYRF_AUTO_CAL_OFFSET, 0x14},											// From manual, needed for initialization
		{CYRF_RX_CFG, CYRF_LNA | CYRF_FAST_TURN_EN},							// Enable low noise amplifier and fast turning
		{CYRF_TX_OFFSET_LSB, 0x55},												// From manual, typical configuration
		{CYRF_TX_OFFSET_MSB, 0x05},												// From manual, typical configuration
		{CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END},						// Force in Synth RX mode
		{CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},	// Enable 64 chip codes, SDR mode and amplifier +4dBm
		{CYRF_DATA64_THOLD, 0x0E},												// From manual, typical configuration
		{CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX},									// Set in Synth RX mode (again, really needed?)
};

const AP_Radio_cypress::config AP_Radio_cypress::cyrf_bind_config[] = {
		{CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},	// Enable 64 chip codes, SDR mode and amplifier +4dBm
		{CYRF_FRAMING_CFG, CYRF_SOP_LEN | 0xE},									// Set SOP CODE to 64 chips and SOP Correlator Threshold to 0xE
		{CYRF_RX_OVERRIDE, CYRF_FRC_RXDR | CYRF_DIS_RXCRC},						// Force receive data rate and disable receive CRC checker
		{CYRF_EOP_CTRL, 0x02},													// Only enable EOP symbol count of 2
		{CYRF_TX_OVERRIDE, CYRF_DIS_TXCRC},										// Disable transmit CRC generate
};
const AP_Radio_cypress::config AP_Radio_cypress::cyrf_transfer_config[] = {
		{CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | CYRF_PA_4},	// Enable 64 chip codes, 8DR mode and amplifier +4dBm
		{CYRF_FRAMING_CFG, CYRF_SOP_EN | CYRF_SOP_LEN | CYRF_LEN_EN | 0xE},		// Set SOP CODE enable, SOP CODE to 64 chips, Packet length enable, and SOP Correlator Threshold to 0xE
		{CYRF_TX_OVERRIDE, 0x00},												// Reset TX overrides
		{CYRF_RX_OVERRIDE, 0x00},												// Reset RX overrides
};

/*
  read radio status, handling the race condition between completion and error
 */
uint8_t AP_Radio_cypress::read_status_debounced(uint8_t adr)
{
    uint8_t ret;

    dev->set_chip_select(true);
    ret = read_register(adr);

    // If COMPLETE and ERROR bits mismatch, then re-read register
    if ((ret & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) != 0 
        && (ret & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) != (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) {
        uint8_t v2;
        dev->read(&v2, 1);
        ret |= v2;   // re-read and make bits sticky
    }
    dev->set_chip_select(false);
    return ret;
}

/*
  force the initial state of the radio
 */
void AP_Radio_cypress::force_initial_state(void)
{
    while (true) {
        write_register(CYRF_XACT_CFG, CYRF_FRC_END);
        uint32_t start_ms = AP_HAL::millis();
        do {
            if ((read_register(CYRF_XACT_CFG) & CYRF_FRC_END) == 0) {
                return;                     // FORCE_END done (osc running)
            }
        } while (AP_HAL::millis() - start_ms < 5);
        
        // FORCE_END failed to complete, implying going SLEEP to IDLE and 
        // oscillator failed to start.  Recover by returning to SLEEP and
        //  trying to start oscillator again.
        write_register(CYRF_XACT_CFG, CYRF_MODE_SLEEP);
    }
}

/*
  set desired channel
 */
void AP_Radio_cypress::set_channel(uint8_t channel)
{
    write_register(CYRF_CHANNEL, channel);
}

void AP_Radio_cypress::radio_set_config(const struct config *conf, uint8_t size)
{
    // setup required radio config
    for (uint8_t i=0; i<size; i++) {
        write_register(conf[i].reg, conf[i].value);
    }
}

/*
  initialise the radio
 */
void AP_Radio_cypress::radio_init(void)
{
    printf("radio_init starting\n");

    // wait for radio to settle
    while (true) {
        uint8_t chan = read_register(CYRF_CHANNEL);
        if (chan == 1) {
            break;
        }
        write_register(CYRF_CHANNEL, 1);
        hal.scheduler->delay(10);
    }

    radio_set_config(cyrf_config, ARRAY_SIZE(cyrf_config));

    radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));
    
    dsm_setup_transfer_dsmx();

    write_register(CYRF_XTAL_CTRL,0x80);  // XOUT=BitSerial
    force_initial_state();
    write_register(CYRF_PWR_CTRL,0x20);   // Disable PMU
    debug("radio_init done\n");

    // setup handler for rising edge of IRQ pin
    //
}

void AP_Radio_cypress::dump_registers(uint8_t n)
{
    for (uint8_t i=0; i<n; i++) {
        uint8_t v = read_register(i);
        printf("%02x:%02x ", i, v);
        if ((i+1) % 16 == 0) {
            printf("\n");
        }
    }
    if (n % 16 != 0) {
        printf("\n");
    }
}

/*
  read one register value
 */
uint8_t AP_Radio_cypress::read_register(uint8_t reg)
{
    uint8_t v = 0;
    (void)dev->read_registers(reg, &v, 1);
    return v;
}


/*
  write multiple bytes
 */
void AP_Radio_cypress::write_multiple(uint8_t reg, uint8_t n, const uint8_t *data)
{
    dev->set_chip_select(true);
    reg |= FLAG_WRITE;
    dev->transfer(&reg, 1, nullptr, 0);
    dev->transfer(data, n, nullptr, 0);
    dev->set_chip_select(false);
}

/*
  write one register value
 */
void AP_Radio_cypress::write_register(uint8_t reg, uint8_t value)
{
    dev->write_register(reg | FLAG_WRITE, value);
}


/*
  transmit a packet of length bytes, blocking until it is complete
*/
bool AP_Radio_cypress::streaming_transmit(const uint8_t *data, uint8_t length)
{
    //struct transmit_state state;

    if (!dev->get_semaphore()->take(0)) {
        return false;
    }

    debug("starting send of %u\n", length);
        
    uint8_t n = MIN(length, 16);

    // pre-fill TX FIFO with up to 16 bytes
    write_register(CYRF_TX_LENGTH, length);
    write_register(CYRF_TX_CTRL, CYRF_TX_CLR);
    if (n > 0) {
        write_multiple(CYRF_TX_BUFFER, n, data);
    }
    data += n;
    length -= n;

    uint8_t irq_bits = CYRF_TX_GO | CYRF_TXC_IRQ | CYRF_TXE_IRQ | CYRF_TXBERR_IRQ;
    if (length > 0) {
        // ask for an interrupt when we can fit more bytes into FIFO
        irq_bits |= CYRF_TXB0_IRQ | CYRF_TXB8_IRQ | CYRF_TXB15_IRQ;
    }

    write_register(CYRF_TX_CTRL, irq_bits);

    bool ret = false;
    
    while (true) {
        wait_irq();
        uint8_t tx_status = read_status_debounced(CYRF_TX_IRQ_STATUS);
        //printf("tx_status: 0x%08x\n", tx_status);
        if (tx_status & (CYRF_TXE_IRQ | CYRF_TXBERR_IRQ | CYRF_TXC_IRQ | CYRF_TXB0_IRQ | CYRF_TXB8_IRQ | CYRF_TXB15_IRQ)) {
            if (tx_status & (CYRF_TXE_IRQ | CYRF_TXBERR_IRQ)) {
                // got an error
                write_register(CYRF_TX_CTRL, CYRF_TX_CLR);
                debug("tx error tx_status=0x%02x\n", (unsigned)tx_status);
                goto done;
            }

            if (tx_status & CYRF_TXC_IRQ) {
                write_register(CYRF_TX_CTRL, CYRF_TX_CLR);
                if (length == 0) {
                    ret = true;
                } else {
                    debug("tx finished with %u left\n", length);
                }
                goto done;
            }
            
            if (tx_status & (CYRF_TXB0_IRQ|CYRF_TXB8_IRQ|CYRF_TXB15_IRQ)) {
                uint8_t space;
                if (tx_status & CYRF_TXB0_IRQ) {
                    space = 16;
                } else if (tx_status & CYRF_TXB8_IRQ) {
                    space = 8;
                } else {
                    space = 1;
                }
                n = MIN(length, space);
                
                if (n == 0) {
                    // clear interrupt bits to stop us getting interrupted
                    write_register(CYRF_TX_CTRL, CYRF_TXC_IRQ | CYRF_TXE_IRQ | CYRF_TXBERR_IRQ);
                } else {
                    write_multiple(CYRF_TX_BUFFER, n, data);
                    
                    length -= n;
                    data += n;
                    
                    if (length == 0) {
                        write_register(CYRF_TX_CTRL, CYRF_TXC_IRQ | CYRF_TXE_IRQ | CYRF_TXBERR_IRQ);
                    }
                }
            }
        }
    }

done:
    debug("TX finished at %u\n", AP_HAL::micros());
    
    dev->get_semaphore()->give();

    return ret;
}


uint8_t AP_Radio_cypress::streaming_receive(uint8_t *data, uint8_t maxlen)
{
    uint8_t *data0 = data;
    uint8_t maxlen0 = maxlen;
    uint8_t irq_bits = CYRF_RXC_IRQ | CYRF_RXE_IRQ | CYRF_RXOW_IRQ | CYRF_RXBERR_IRQ;
    if (maxlen > 16) {
        irq_bits |= CYRF_RXB16_IRQ | CYRF_RXB8_IRQ;
    }
    write_register(CYRF_RX_CTRL, CYRF_RX_GO | irq_bits);
    uint8_t rx_status;
    uint8_t ret = 0;
    while (true) {
        wait_irq();
        rx_status = read_status_debounced(CYRF_RX_IRQ_STATUS);
        if (rx_status == 0) {
            continue;
        }
        //printf("rx_status 0x%02x\n", rx_status);
        if (rx_status & CYRF_RXOW_IRQ) {
            write_register(CYRF_RX_IRQ_STATUS, CYRF_RXOW_IRQ);
        }
        if ((rx_status & (CYRF_RXB16_IRQ | CYRF_RXB8_IRQ))) {
            uint8_t avail;
            if (rx_status & CYRF_RXB16_IRQ) {
                avail = 16;
            } else if (rx_status & CYRF_RXB8_IRQ) {
                avail = 8;
            } else {
                // don't try and read single bytes here. The read
                // overhead is high enough for a single byte that we
                // are much better off waiting for at least 8 bytes to
                // be in the FIFO
                avail = 0;
            }
            uint8_t n = MIN(maxlen, avail);
            if (n > 0) {
                dev->read_registers(CYRF_RX_BUFFER, data, n);
                data += n;
                maxlen -= n;
                ret += n;
            }
        }
        if (rx_status & CYRF_RXC_IRQ) {
            break;
        }
        if (rx_status & (CYRF_RXBERR_IRQ | CYRF_RXE_IRQ)) {
            if (ret > 0) {
                debug("rx_status=0x%02x clearing %u bytes\n", rx_status, ret);
            }
            // discard the partial packet, if any
            data = data0;
            maxlen = maxlen0;
            ret = 0;

            // when we get a buffer error we don't really know why it
            // happened. To maximise the chances of clearing the error
            // we read the complete FIFO length to ensure the FIFO is
            // clear
            uint8_t tmp[16];
            dev->read_registers(CYRF_RX_BUFFER, tmp, 16);
            rx_status = read_status_debounced(CYRF_RX_IRQ_STATUS);

            // restart the receive
            write_register(CYRF_RX_CTRL, CYRF_RX_GO | irq_bits);
            continue;
        }
    }

    uint8_t rlen = read_register(CYRF_RX_LENGTH);
#if RADIO_DEBUG
    uint8_t rem = read_register(CYRF_RX_COUNT);
    uint8_t ret0 = ret;
#endif
    
    uint8_t n = rlen - ret;
    if (n > maxlen) {
        n = maxlen;
    }
    if (n > 0) {
        dev->read_registers(CYRF_RX_BUFFER, data, n);
        ret += n;
    }

    // Write desired End State using FORCE_END_STATE feature
    do {
        write_register(CYRF_XACT_CFG, CYRF_FRC_END);
        // Wait for FORCE_END to complete
        hal.scheduler->delay_microseconds(100);
    } while ((read_register(CYRF_XACT_CFG) & CYRF_FRC_END) != 0);    
    
    debug("receive ret=%u ret0=%u rlen=%u rem=%u rx_status=0x%02x\n", ret, ret0, rlen, rem, rx_status);
    return ret;
}

/*
  wait for an interrupt from the radio
 */
void AP_Radio_cypress::wait_irq(void)
{
    sem_init(&irq_sem, 0, 0);
    stm32_gpiosetevent(GPIO_GPIO4_INPUT, true, false, false, irq_trampoline);
    if (!stm32_gpioread(GPIO_GPIO4_INPUT)) {
        sem_wait(&irq_sem);
    } else {
        // we've hit a race condition where the IRQ may have been
        // raised before irq_trampoline was setup. Just clear the
        // handler
        stm32_gpiosetevent(GPIO_GPIO4_INPUT, false, false, false, nullptr);
    }
}


/*
  called on rising edge of radio IRQ pin
 */
int AP_Radio_cypress::irq_trampoline(int irq, void *context)
{
    stm32_gpiosetevent(GPIO_GPIO4_INPUT, false, false, false, nullptr);
    sem_post(&radio_instance->irq_sem);
    return 0;
}

/*
 Set the current DSM channel with SOP, CRC and data code
 */
void AP_Radio_cypress::dsm_set_channel(uint8_t channel, bool is_dsm2, uint8_t sop_col, uint8_t data_col, uint16_t crc_seed)
{
	uint8_t pn_row;
	pn_row = is_dsm2? channel % 5 : (channel-2) % 5;

    // set CRC seed
	write_register(CYRF_CRC_SEED_LSB, crc_seed & 0xff);
	write_register(CYRF_CRC_SEED_MSB, crc_seed >> 8);

    // set start of packet code
    write_multiple(CYRF_SOP_CODE, sizeof(pn_codes[0][0]), pn_codes[pn_row][sop_col]);

    // set data code
    write_multiple(CYRF_DATA_CODE, sizeof(pn_codes[0][0]), pn_codes[pn_row][data_col]);

	// Change channel
	set_channel(channel);
}

/*
  Generate the DSMX channels from the manufacturer ID
 */
void AP_Radio_cypress::dsm_generate_channels_dsmx(uint8_t mfg_id[4], uint8_t channels[23])
{
	// Calculate the DSMX channels
	int idx = 0;
	uint32_t id = ~((mfg_id[0] << 24) | (mfg_id[1] << 16) |
                    (mfg_id[2] << 8) | (mfg_id[3] << 0));
	uint32_t id_tmp = id;
    
	// While not all channels are set
	while(idx < 23) {
		int i;
		int count_3_27 = 0, count_28_51 = 0, count_52_76 = 0;

		id_tmp = id_tmp * 0x0019660D + 0x3C6EF35F; // Randomization
		uint8_t next_ch = ((id_tmp >> 8) % 0x49) + 3;       // Use least-significant byte and must be larger than 3
		if (((next_ch ^ id) & 0x01 ) == 0) {
			continue;
        }

		// Go trough all already set channels
		for (i = 0; i < idx; i++) {
			// Channel is already used
			if (channels[i] == next_ch) {
				break;
            }
            
			// Count the channel groups
			if(channels[i] <= 27) {
				count_3_27++;
			} else if (channels[i] <= 51) {
				count_28_51++;
            } else {
				count_52_76++;
            }
		}

		// When channel is already used continue
		if (i != idx) {
			continue;
        }

		// Set the channel when channel groups aren't full
		if ((next_ch < 28 && count_3_27 < 8)						// Channels 3-27: max 8
            || (next_ch >= 28 && next_ch < 52 && count_28_51 < 7)	// Channels 28-52: max 7
            || (next_ch >= 52 && count_52_76 < 8)) {				// Channels 52-76: max 8
			channels[idx++] = next_ch;
		}
	}

    printf("DSM generated channels for %02x:%02x:%02x:%02x\n",
           dsm.mfg_id[0], dsm.mfg_id[1], dsm.mfg_id[2], dsm.mfg_id[3]);
}

/*
  setup for DSMX transfers
 */
void AP_Radio_cypress::dsm_setup_transfer_dsmx(void)
{
    dsm.current_channel = 0;

    dsm.crc_seed = ~((dsm.mfg_id[0] << 8) + dsm.mfg_id[1]);
    dsm.sop_col = (dsm.mfg_id[0] + dsm.mfg_id[1] + dsm.mfg_id[2] + 2) & 0x07;
	dsm.data_col = 7 - dsm.sop_col;

    printf("sop_col:%u data_col:%u crc_seed:0x%04x\n",
           dsm.sop_col, dsm.data_col, dsm.crc_seed);

    dsm_generate_channels_dsmx(dsm.mfg_id, dsm.channels);
    dsm.current_channel = 22;
    dsm_set_next_channel();
}

/*
  move to the next DSM channnel
 */
void AP_Radio_cypress::dsm_set_next_channel(void)
{
	dsm.current_channel = dsm.is_dsm2? (dsm.current_channel+1) % 2 : (dsm.current_channel+1) % 23;
	dsm.crc_seed		= ~dsm.crc_seed;
	dsm_set_channel(dsm.channels[dsm.current_channel], dsm.is_dsm2,
                    dsm.sop_col, dsm.data_col, dsm.crc_seed);
}

