#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <board_config.h>
#include "AP_Radio_cypress.h"
#include <utility>
#include <stdio.h>
#include <StorageManager/StorageManager.h>
#include <AP_HAL/utility/dsm.h>

/*
  driver for CYRF6936 radio

  Many thanks to the SuperBitRF project from Paparrazi for their DSM
  configuration code and register defines
   https://github.com/esden/superbitrf-firmware
 */

#ifndef CYRF_SPI_DEVICE
# define CYRF_SPI_DEVICE "cypress"
#endif

#ifndef CYRF_IRQ_INPUT
# define CYRF_IRQ_INPUT (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)
#endif

#ifndef CYRF_RESET_PIN
# define CYRF_RESET_PIN (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_EXTI|GPIO_PORTB|GPIO_PIN0)
#endif

extern const AP_HAL::HAL& hal;

#define debug(level, fmt, args...)   do { if ((level) <= get_debug_level()) { printf(fmt, ##args); }} while (0)

#define LP_FIFO_SIZE  16      // Physical data FIFO lengths in Radio

/* The SPI interface defines */
enum {
    CYRF_CHANNEL            = 0x00,
    CYRF_TX_LENGTH          = 0x01,
    CYRF_TX_CTRL            = 0x02,
    CYRF_TX_CFG             = 0x03,
    CYRF_TX_IRQ_STATUS      = 0x04,
    CYRF_RX_CTRL            = 0x05,
    CYRF_RX_CFG             = 0x06,
    CYRF_RX_IRQ_STATUS      = 0x07,
    CYRF_RX_STATUS          = 0x08,
    CYRF_RX_COUNT           = 0x09,
    CYRF_RX_LENGTH          = 0x0A,
    CYRF_PWR_CTRL           = 0x0B,
    CYRF_XTAL_CTRL          = 0x0C,
    CYRF_IO_CFG             = 0x0D,
    CYRF_GPIO_CTRL          = 0x0E,
    CYRF_XACT_CFG           = 0x0F,
    CYRF_FRAMING_CFG        = 0x10,
    CYRF_DATA32_THOLD       = 0x11,
    CYRF_DATA64_THOLD       = 0x12,
    CYRF_RSSI               = 0x13,
    CYRF_EOP_CTRL           = 0x14,
    CYRF_CRC_SEED_LSB       = 0x15,
    CYRF_CRC_SEED_MSB       = 0x16,
    CYRF_TX_CRC_LSB         = 0x17,
    CYRF_TX_CRC_MSB         = 0x18,
    CYRF_RX_CRC_LSB         = 0x19,
    CYRF_RX_CRC_MSB         = 0x1A,
    CYRF_TX_OFFSET_LSB      = 0x1B,
    CYRF_TX_OFFSET_MSB      = 0x1C,
    CYRF_MODE_OVERRIDE      = 0x1D,
    CYRF_RX_OVERRIDE        = 0x1E,
    CYRF_TX_OVERRIDE        = 0x1F,
    CYRF_TX_BUFFER          = 0x20,
    CYRF_RX_BUFFER          = 0x21,
    CYRF_SOP_CODE           = 0x22,
    CYRF_DATA_CODE          = 0x23,
    CYRF_PREAMBLE           = 0x24,
    CYRF_MFG_ID             = 0x25,
    CYRF_XTAL_CFG           = 0x26,
    CYRF_CLK_OFFSET         = 0x27,
    CYRF_CLK_EN             = 0x28,
    CYRF_RX_ABORT           = 0x29,
    CYRF_AUTO_CAL_TIME      = 0x32,
    CYRF_AUTO_CAL_OFFSET    = 0x35,
    CYRF_ANALOG_CTRL        = 0x39,
};
#define CYRF_DIR                (1<<7) /**< Bit for enabling writing */

// CYRF_MODE_OVERRIDE
#define CYRF_RST                (1<<0)

// CYRF_CLK_EN
#define CYRF_RXF                (1<<1)

// CYRF_XACT_CFG
enum {
    CYRF_MODE_SLEEP     = (0x0<<2),
    CYRF_MODE_IDLE      = (0x1<<2),
    CYRF_MODE_SYNTH_TX  = (0x2<<2),
    CYRF_MODE_SYNTH_RX  = (0x3<<2),
    CYRF_MODE_RX        = (0x4<<2),
};
#define CYRF_FRC_END      (1<<5)
#define CYRF_ACK_EN       (1<<7)

// CYRF_IO_CFG
#define CYRF_IRQ_GPIO     (1<<0)
#define CYRF_SPI_3PIN     (1<<1)
#define CYRF_PACTL_GPIO   (1<<2)
#define CYRF_PACTL_OD     (1<<3)
#define CYRF_XOUT_OD      (1<<4)
#define CYRF_MISO_OD      (1<<5)
#define CYRF_IRQ_POL      (1<<6)
#define CYRF_IRQ_OD       (1<<7)

// CYRF_FRAMING_CFG
#define CYRF_LEN_EN       (1<<5)
#define CYRF_SOP_LEN      (1<<6)
#define CYRF_SOP_EN       (1<<7)

// CYRF_RX_STATUS
enum {
    CYRF_RX_DATA_MODE_GFSK  = 0x00,
    CYRF_RX_DATA_MODE_8DR   = 0x01,
    CYRF_RX_DATA_MODE_DDR   = 0x10,
    CYRF_RX_DATA_MODE_NV    = 0x11,
};
#define CYRF_RX_CODE            (1<<2)
#define CYRF_BAD_CRC     (1<<3)
#define CYRF_CRC0        (1<<4)
#define CYRF_EOP_ERR     (1<<5)
#define CYRF_PKT_ERR     (1<<6)
#define CYRF_RX_ACK      (1<<7)

// CYRF_TX_IRQ_STATUS
#define CYRF_TXE_IRQ     (1<<0)
#define CYRF_TXC_IRQ     (1<<1)
#define CYRF_TXBERR_IRQ  (1<<2)
#define CYRF_TXB0_IRQ    (1<<3)
#define CYRF_TXB8_IRQ    (1<<4)
#define CYRF_TXB15_IRQ   (1<<5)
#define CYRF_LV_IRQ      (1<<6)
#define CYRF_OS_IRQ      (1<<7)

// CYRF_RX_IRQ_STATUS
#define CYRF_RXE_IRQ     (1<<0)
#define CYRF_RXC_IRQ     (1<<1)
#define CYRF_RXBERR_IRQ  (1<<2)
#define CYRF_RXB1_IRQ    (1<<3)
#define CYRF_RXB8_IRQ    (1<<4)
#define CYRF_RXB16_IRQ   (1<<5)
#define CYRF_SOPDET_IRQ  (1<<6)
#define CYRF_RXOW_IRQ    (1<<7)

// CYRF_TX_CTRL
#define CYRF_TXE_IRQEN    (1<<0)
#define CYRF_TXC_IRQEN    (1<<1)
#define CYRF_TXBERR_IRQEN (1<<2)
#define CYRF_TXB0_IRQEN   (1<<3)
#define CYRF_TXB8_IRQEN   (1<<4)
#define CYRF_TXB15_IRQEN  (1<<5)
#define CYRF_TX_CLR       (1<<6)
#define CYRF_TX_GO        (1<<7)

// CYRF_RX_CTRL
#define CYRF_RXE_IRQEN    (1<<0)
#define CYRF_RXC_IRQEN    (1<<1)
#define CYRF_RXBERR_IRQEN (1<<2)
#define CYRF_RXB1_IRQEN   (1<<3)
#define CYRF_RXB8_IRQEN   (1<<4)
#define CYRF_RXB16_IRQEN  (1<<5)
#define CYRF_RSVD         (1<<6)
#define CYRF_RX_GO        (1<<7)

// CYRF_RX_OVERRIDE
#define CYRF_ACE          (1<<1)
#define CYRF_DIS_RXCRC    (1<<2)
#define CYRF_DIS_CRC0     (1<<3)
#define CYRF_FRC_RXDR     (1<<4)
#define CYRF_MAN_RXACK    (1<<5)
#define CYRF_RXTX_DLY     (1<<6)
#define CYRF_ACK_RX       (1<<7)

// CYRF_TX_OVERRIDE
#define CYRF_TX_INV       (1<<0)
#define CYRF_DIS_TXCRC    (1<<2)
#define CYRF_OVRD_ACK     (1<<3)
#define CYRF_MAN_TXACK    (1<<4)
#define CYRF_FRC_PRE      (1<<6)
#define CYRF_ACK_TX       (1<<7)

// CYRF_RX_CFG
#define CYRF_VLD_EN       (1<<0)
#define CYRF_RXOW_EN      (1<<1)
#define CYRF_FAST_TURN_EN (1<<3)
#define CYRF_HILO         (1<<4)
#define CYRF_ATT          (1<<5)
#define CYRF_LNA          (1<<6)
#define CYRF_AGC_EN       (1<<7)

// CYRF_TX_CFG
enum {
    CYRF_PA_M35      = 0x0,
    CYRF_PA_M30      = 0x1,
    CYRF_PA_M24      = 0x2,
    CYRF_PA_M18      = 0x3,
    CYRF_PA_M13      = 0x4,
    CYRF_PA_M5       = 0x5,
    CYRF_PA_0        = 0x6,
    CYRF_PA_4        = 0x7,
};
enum {
    CYRF_DATA_MODE_GFSK   = (0x0 <<3),
    CYRF_DATA_MODE_8DR    = (0x1 <<3),
    CYRF_DATA_MODE_DDR    = (0x2 <<3),
    CYRF_DATA_MODE_SDR    = (0x3 <<3),
};
#define CYRF_DATA_CODE_LENGTH    (1<<5)


#define FLAG_WRITE      0x80
#define FLAG_AUTO_INC   0x40

#define DSM_MAX_CHANNEL 0x4F

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
    dev = std::move(hal.spi->get_device(CYRF_SPI_DEVICE));

    load_bind_info();
    
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
      to reset radio hold reset high for 0.5s, then low for 0.5s
     */
    stm32_configgpio(CYRF_RESET_PIN);
    stm32_gpiowrite(CYRF_RESET_PIN, 1);
    hal.scheduler->delay(500);
    stm32_gpiowrite(CYRF_RESET_PIN, 0);
    hal.scheduler->delay(500);

    // use AUX5 as radio IRQ pin
    stm32_configgpio(CYRF_IRQ_INPUT);

    radio_init();
    dev->get_semaphore()->give();

    if (dsm.protocol == DSM_NONE) {
        start_recv_bind();
    }
    
    return true;
}

/*
  return statistics structure from radio
 */
const AP_Radio::stats &AP_Radio_cypress::get_stats(void)
{
    return stats;
}

/*
  read one pwm channel from radio
 */
uint16_t AP_Radio_cypress::read(uint8_t chan)
{
    if (dsm.need_bind_save) {
        save_bind_info();
    }
    if (chan >= max_channels) {
        return 0;
    }
    return dsm.pwm_channels[chan];
}

/*
  print one second debug info
 */
void AP_Radio_cypress::print_debug_info(void)
{
    debug(2, "recv:%3u bad:%3u to:%3u re:%u N:%2u 1:%4u 1:%4u 3:%4u 4:%4u 5:%4u 6:%4u 7:%4u 8:%4u 14:%u\n",
          stats.recv_packets - last_stats.recv_packets,
          stats.bad_packets - last_stats.bad_packets,
          stats.timeouts - last_stats.timeouts,
          stats.recv_errors - last_stats.recv_errors,
          num_channels(),
          dsm.pwm_channels[0], dsm.pwm_channels[1], dsm.pwm_channels[2], dsm.pwm_channels[3], 
          dsm.pwm_channels[4], dsm.pwm_channels[5], dsm.pwm_channels[6], dsm.pwm_channels[7],
          dsm.pwm_channels[13]);
}

/*
  return number of active channels
 */
uint8_t AP_Radio_cypress::num_channels(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_debug_print_ms > 1000) {
        last_debug_print_ms = now;
        if (get_debug_level() > 1) {
            print_debug_info();
        }

        uint8_t ch = get_rssi_chan();
        if (ch > 0) {
            dsm.pwm_channels[ch-1] = (uint8_t)dsm.rssi;
            dsm.num_channels = MAX(dsm.num_channels, ch);
        }

        ch = get_rate_chan();
        if (ch > 0) {
            dsm.pwm_channels[ch-1] = stats.recv_packets - last_stats.recv_packets;
            dsm.num_channels = MAX(dsm.num_channels, ch);
        }
        last_stats = stats;
    }
    return dsm.num_channels;
}

/*
  return time of last receive in microseconds
 */
uint32_t AP_Radio_cypress::last_recv_us(void)
{
    return dsm.last_recv_us;
}

/*
  send len bytes as a single packet
 */
bool AP_Radio_cypress::send(const uint8_t *pkt, uint16_t len)
{
    // disabled for now
    return false;
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
        {CYRF_MODE_OVERRIDE, CYRF_RST},                                         // Reset the device
        {CYRF_CLK_EN, CYRF_RXF},                                                // Enable the clock
        {CYRF_AUTO_CAL_TIME, 0x3C},                                             // From manual, needed for initialization
        {CYRF_AUTO_CAL_OFFSET, 0x14},                                           // From manual, needed for initialization
        {CYRF_RX_CFG, CYRF_LNA | CYRF_FAST_TURN_EN},                            // Enable low noise amplifier and fast turning
        {CYRF_TX_OFFSET_LSB, 0x55},                                             // From manual, typical configuration
        {CYRF_TX_OFFSET_MSB, 0x05},                                             // From manual, typical configuration
        {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END},                     // Force in Synth RX mode
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},  // Enable 64 chip codes, SDR mode and amplifier +4dBm
        {CYRF_DATA64_THOLD, 0x0E},                                              // From manual, typical configuration
        {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX},                                    // Set in Synth RX mode (again, really needed?)
        {CYRF_IO_CFG, CYRF_IRQ_POL},                                            // IRQ active high
};

const AP_Radio_cypress::config AP_Radio_cypress::cyrf_bind_config[] = {
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},   // Enable 64 chip codes, SDR mode and amplifier +4dBm
        {CYRF_FRAMING_CFG, CYRF_SOP_LEN | 0xE},                                  // Set SOP CODE to 64 chips and SOP Correlator Threshold to 0xE
        {CYRF_RX_OVERRIDE, CYRF_FRC_RXDR | CYRF_DIS_RXCRC},                      // Force receive data rate and disable receive CRC checker
        {CYRF_EOP_CTRL, 0x02},                                                   // Only enable EOP symbol count of 2
        {CYRF_TX_OVERRIDE, CYRF_DIS_TXCRC},                                      // Disable transmit CRC generate
};
const AP_Radio_cypress::config AP_Radio_cypress::cyrf_transfer_config[] = {
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | CYRF_PA_4},   // Enable 64 chip codes, 8DR mode and amplifier +4dBm
        {CYRF_FRAMING_CFG, CYRF_SOP_EN | CYRF_SOP_LEN | CYRF_LEN_EN | 0xE},      // Set SOP CODE enable, SOP CODE to 64 chips, Packet length enable, and SOP Correlator Threshold to 0xE
        {CYRF_TX_OVERRIDE, 0x00},                                                // Reset TX overrides
        {CYRF_RX_OVERRIDE, 0x00},                                                // Reset RX overrides
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
    //printf("chan %u\n", channel);
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
    debug(1, "Cypress: radio_init starting\n");

    // wait for radio to settle
    while (true) {
        uint8_t chan = read_register(CYRF_CHANNEL);
        if (chan == 1) {
            break;
        }
        write_register(CYRF_CHANNEL, 1);
        hal.scheduler->delay(10);
    }

    // base config
    radio_set_config(cyrf_config, ARRAY_SIZE(cyrf_config));

    // start with receive config
    radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));

    if (get_disable_crc()) {
        write_register(CYRF_RX_OVERRIDE, CYRF_DIS_RXCRC);
    }
    
    dsm_setup_transfer_dsmx();

    write_register(CYRF_XTAL_CTRL,0x80);  // XOUT=BitSerial
    force_initial_state();
    write_register(CYRF_PWR_CTRL,0x20);   // Disable PMU

    // start in RECV state
    state = STATE_RECV;

    debug(1, "Cypress: radio_init done\n");

    start_receive();

    // setup handler for rising edge of IRQ pin
    stm32_gpiosetevent(CYRF_IRQ_INPUT, true, false, false, irq_radio_trampoline);
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
  parse channels from a packet
 */
void AP_Radio_cypress::parse_dsm_channels(const uint8_t *data)
{
    dsm_decode(AP_HAL::micros64(),
               data,
               dsm.pwm_channels,
               &dsm.num_channels,
               ARRAY_SIZE(dsm.pwm_channels));
}

/*
  process an incoming bind packet
 */
void AP_Radio_cypress::process_bind(const uint8_t *pkt, uint8_t len)
{
    if (len != 16) {
        return;
    }
    bool ok = (len==16 && pkt[0] == pkt[4] && pkt[1] == pkt[5] && pkt[2] == pkt[6] && pkt[3] == pkt[7]);

    // Calculate the first sum
    uint16_t bind_sum = 384 - 0x10;
    for (uint8_t i = 0; i < 8; i++) {
        bind_sum += pkt[i];
    }

    // Check the first sum
    if (pkt[8] != bind_sum >> 8 || pkt[9] != (bind_sum & 0xFF)) {
        ok = false;
    }

    // Calculate second sum
    for (uint8_t i = 8; i < 14; i++) {
        bind_sum += pkt[i];
    }

    // Check the second sum
    if (pkt[14] != bind_sum >> 8 || pkt[15] != (bind_sum & 0xFF)) {
        ok = false;
    }

    if (ok) {
        uint8_t mfg_id[4] = {uint8_t(~pkt[0]), uint8_t(~pkt[1]), uint8_t(~pkt[2]), uint8_t(~pkt[3])};
        uint8_t num_chan = pkt[11];
        uint8_t protocol = pkt[12];
        
        // change to normal receive
        memcpy(dsm.mfg_id, mfg_id, 4);
        state = STATE_RECV;

        radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));

        if (get_disable_crc()) {
            write_register(CYRF_RX_OVERRIDE, CYRF_DIS_RXCRC);
        }

        dsm_setup_transfer_dsmx();
        dsm.protocol = (enum dsm_protocol)protocol;

        debug(1, "BIND OK: mfg_id={0x%02x, 0x%02x, 0x%02x, 0x%02x} N=%u P=0x%02x DSM2=%u\n",
              mfg_id[0], mfg_id[1], mfg_id[2], mfg_id[3],
              num_chan,
              protocol,
              is_DSM2());
        
        dsm.need_bind_save = true;
    }
}

/*
  process an incoming packet
 */
void AP_Radio_cypress::process_packet(const uint8_t *pkt, uint8_t len)
{
    if (len == 16) {
        bool ok;
        const uint8_t *id = dsm.mfg_id;
        if (is_DSM2()) {
            ok = (pkt[0] == ((~id[2])&0xFF) && pkt[1] == (~id[3]&0xFF));
        } else {
            ok = (pkt[0] == id[2] && pkt[1] == id[3]);
        }
        if (ok && is_DSM2() && dsm.sync < DSM2_OK) {
            if (dsm.sync == DSM2_SYNC_A) {
                dsm.channels[0] = dsm.current_rf_channel;
                dsm.sync = DSM2_SYNC_B;
                debug(2, "DSM2 SYNCA chan=%u\n", dsm.channels[0]);
            } else {
                if (dsm.current_rf_channel != dsm.channels[0]) {
                    dsm.channels[1] = dsm.current_rf_channel;
                    dsm.sync = DSM2_OK;                    
                    debug(2, "DSM2 SYNCB chan=%u\n", dsm.channels[1]);
                }
            }
        }
        if (ok) {
            dsm.last_recv_chan = dsm.current_channel;
            dsm.last_recv_us = AP_HAL::micros();
            if (dsm.crc_errors > 2) {
                dsm.crc_errors -= 2;
            }

            parse_dsm_channels(pkt);

            stats.recv_packets++;

            // sample the RSSI
            uint8_t rssi = read_register(CYRF_RSSI);
            dsm.rssi = 0.95 * dsm.rssi + 0.05 * rssi;
        } else {
            stats.bad_packets++;
        }
    } else {
            stats.bad_packets++;
    }
}


/*
  start packet receive
 */
void AP_Radio_cypress::start_receive(void)
{
    dsm_choose_channel();
    
    write_register(CYRF_RX_IRQ_STATUS, CYRF_RXOW_IRQ);
    write_register(CYRF_RX_CTRL, CYRF_RX_GO | CYRF_RXC_IRQEN | CYRF_RXE_IRQEN);

    dsm.receive_start_us = AP_HAL::micros();
    if (state == STATE_BIND) {
        dsm.receive_timeout_usec = 15000;
    } else {
        dsm.receive_timeout_usec = 23000;
    }
    //dsm.receive_timeout_usec = 45000;
    hrt_call_after(&wait_call, dsm.receive_timeout_usec, (hrt_callout)irq_timeout_trampoline, nullptr);
}

/*
  handle a receive IRQ
 */
void AP_Radio_cypress::irq_handler_recv(uint8_t rx_status)
{
    if ((rx_status & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) == 0) {
        // nothing interesting yet
        return;
    }

    uint8_t pkt[16];
    uint8_t rlen = read_register(CYRF_RX_COUNT);
    if (rlen > 16) {
        rlen = 16;
    }
    if (rlen > 0) {
        dev->read_registers(CYRF_RX_BUFFER, pkt, rlen);
    }

    if (rx_status & CYRF_RXE_IRQ) {
        uint8_t reason = read_register(CYRF_RX_STATUS);
        //hal.console->printf("reason=%u\n", reason);
        if (reason & CYRF_BAD_CRC) {
            dsm.crc_errors++;
            if (dsm.crc_errors > 20) {
                debug(2, "Flip CRC\n");
                // flip crc seed, this allows us to resync with transmitter
                dsm.crc_seed = ~dsm.crc_seed;
                dsm.crc_errors = 0;
            }
        }
        write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
        write_register(CYRF_RX_ABORT, 0);
        stats.recv_errors++;
    } else if (rx_status & CYRF_RXC_IRQ) {
        if (state == STATE_RECV) {
            process_packet(pkt, rlen);
        } else {
            process_bind(pkt, rlen);
        }
    }

    start_receive();
}


/*
  IRQ handler
 */
void AP_Radio_cypress::irq_handler(void)
{
    //hal.console->printf("IRQ\n");
    if (!dev->get_semaphore()->take_nonblocking()) {
        // we have to wait for timeout instead
        return;
    }
    // always read both rx and tx status. This ensure IRQ is cleared
    uint8_t rx_status = read_status_debounced(CYRF_RX_IRQ_STATUS);
    read_status_debounced(CYRF_TX_IRQ_STATUS);

    switch (state) {
    case STATE_RECV:
    case STATE_BIND:
        irq_handler_recv(rx_status);
        break;
        
    default:
        break;
    }
    dev->get_semaphore()->give();
}

/*
  called on radio timeout
 */
void AP_Radio_cypress::irq_timeout(void)
{
    stats.timeouts++;
    if (!dev->get_semaphore()->take_nonblocking()) {
        // schedule a new timeout
        hrt_call_after(&wait_call, dsm.receive_timeout_usec, (hrt_callout)irq_timeout_trampoline, nullptr);
        return;
    }

    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);

    start_receive();

    dev->get_semaphore()->give();
}


/*
  called on rising edge of radio IRQ pin
 */
int AP_Radio_cypress::irq_radio_trampoline(int irq, void *context)
{
    radio_instance->irq_handler();
    return 0;
}


/*
  called on HRT timeout
 */
int AP_Radio_cypress::irq_timeout_trampoline(int irq, void *context)
{
    radio_instance->irq_timeout();
    return 0;
}

/*
 Set the current DSM channel with SOP, CRC and data code
 */
void AP_Radio_cypress::dsm_set_channel(uint8_t channel, bool is_dsm2, uint8_t sop_col, uint8_t data_col, uint16_t crc_seed)
{
    //printf("dsm_set_channel: %u\n", channel);

    uint8_t pn_row;
    pn_row = is_dsm2? channel % 5 : (channel-2) % 5;

    // set CRC seed
    write_register(CYRF_CRC_SEED_LSB, crc_seed & 0xff);
    write_register(CYRF_CRC_SEED_MSB, crc_seed >> 8);

    // set start of packet code
    if (memcmp(dsm.last_sop_code, pn_codes[pn_row][sop_col], 8) != 0) {
        write_multiple(CYRF_SOP_CODE, 8, pn_codes[pn_row][sop_col]);
        memcpy(dsm.last_sop_code, pn_codes[pn_row][sop_col], 8);
    }

    // set data code
    if (memcmp(dsm.last_data_code, pn_codes[pn_row][data_col], 16) != 0) {
        write_multiple(CYRF_DATA_CODE, 16, pn_codes[pn_row][data_col]);
        memcpy(dsm.last_data_code, pn_codes[pn_row][data_col], 16);
    }

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
        if ((next_ch < 28 && count_3_27 < 8)                        // Channels 3-27: max 8
            || (next_ch >= 28 && next_ch < 52 && count_28_51 < 7)    // Channels 28-52: max 7
            || (next_ch >= 52 && count_52_76 < 8)) {                // Channels 52-76: max 8
            channels[idx++] = next_ch;
        }
    }

    debug(2, "Generated DSMX channels\n");
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

    dsm_generate_channels_dsmx(dsm.mfg_id, dsm.channels);
}

/*
  choose channel to receive on
 */
void AP_Radio_cypress::dsm_choose_channel(void)
{
    uint32_t now = AP_HAL::micros();
    uint32_t dt = now - dsm.last_recv_us;
    
    const uint32_t cycle_time = 11000;
    uint8_t next_channel;

    
    if (state == STATE_BIND) {
        if (now - dsm.last_chan_change_us > 15000) {
            // always use odd channel numbers for bind
            dsm.current_rf_channel |= 1;
            dsm.current_rf_channel = (dsm.current_rf_channel+2) % DSM_MAX_CHANNEL;
            dsm.last_chan_change_us = now;
        }
        set_channel(dsm.current_rf_channel);
        return;
    }

    if (is_DSM2() && dsm.sync < DSM2_OK) {
        if (now - dsm.last_chan_change_us > 15000) {
            dsm.current_rf_channel = (dsm.current_rf_channel+1) % DSM_MAX_CHANNEL;
            dsm.last_chan_change_us = now;
        }
        //hal.console->printf("%u chan=%u\n", AP_HAL::micros(), dsm.current_rf_channel);
        dsm_set_channel(dsm.current_rf_channel, is_DSM2(),
                        dsm.sop_col, dsm.data_col,
                        dsm.sync==DSM2_SYNC_B?~dsm.crc_seed:dsm.crc_seed);
        return;
    }
    
    if (dt < 1000) {
        // normal channel advance
        next_channel = dsm.last_recv_chan + 1;
    } else if (dt > 10*cycle_time) {
        // stay with this channel till transmitter finds us
        next_channel = dsm.last_recv_chan + (dt % 15);
    } else {
        // predict next channel
        next_channel = dsm.last_recv_chan + 1;
        next_channel += (dt / cycle_time) * 2;
        if (dt % cycle_time > 5000) {
            next_channel++;
        }
    }

    uint8_t chan_count = is_DSM2()?2:23;
    dsm.current_channel = next_channel;
    if (dsm.current_channel >= chan_count) {
        dsm.current_channel %= chan_count;
        if (!is_DSM2()) {
            dsm.crc_seed = ~dsm.crc_seed;
        }
    }
    
    dsm.current_rf_channel = dsm.channels[dsm.current_channel];

    uint16_t seed = dsm.crc_seed;
    if (dsm.current_channel & 1) {
        seed = ~seed;
    }

    if (is_DSM2()) {
        if (now - dsm.last_recv_us > 5000000) {
            debug(2, "DSM2 resync\n");
            dsm.sync = DSM2_SYNC_A;
        }
    }
    
    dsm_set_channel(dsm.current_rf_channel, is_DSM2(),
                    dsm.sop_col, dsm.data_col, seed);
}

/*
  setup radio for bind
 */
void AP_Radio_cypress::start_recv_bind(void)
{
    if (!dev->get_semaphore()->take(0)) {
        // shouldn't be possible
        return;
    }

    debug(1, "Cypress: start_recv_bind\n");

    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    
    state = STATE_BIND;

    radio_set_config(cyrf_bind_config, ARRAY_SIZE(cyrf_bind_config));

    write_register(CYRF_CRC_SEED_LSB, 0);
    write_register(CYRF_CRC_SEED_MSB, 0);

    write_multiple(CYRF_SOP_CODE, 8, pn_codes[0][0]);

    uint8_t data_code[16];
    memcpy(data_code, pn_codes[0][8], 8);
    memcpy(&data_code[8], pn_bind, 8);
    write_multiple(CYRF_DATA_CODE, 16, data_code);

    dsm.current_rf_channel = 1;

    start_receive();

    dev->get_semaphore()->give();
}

/*
  save bind info
 */
void AP_Radio_cypress::save_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;

    info.magic = bind_magic;
    memcpy(info.mfg_id, dsm.mfg_id, sizeof(info.mfg_id));
    info.protocol = dsm.protocol;

    if (bind_storage.write_block(0, &info, sizeof(info))) {
        dsm.need_bind_save = false;
    }
}

/*
  load bind info
 */
void AP_Radio_cypress::load_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;

    if (bind_storage.read_block(&info, 0, sizeof(info)) && info.magic == bind_magic) {
        memcpy(dsm.mfg_id, info.mfg_id, sizeof(info.mfg_id));
        dsm.protocol = info.protocol;
    }
}

bool AP_Radio_cypress::is_DSM2(void)
{
    if (get_protocol() != AP_Radio::PROTOCOL_AUTO) {
        return get_protocol() == AP_Radio::PROTOCOL_DSM2;
    }
    return dsm.protocol == DSM_DSM2_1 || dsm.protocol == DSM_DSM2_2;
}
