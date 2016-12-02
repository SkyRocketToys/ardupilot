#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <board_config.h>
#include "AP_Radio_cypress.h"
#include <utility>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define RADIO_DEBUG 0
#if RADIO_DEBUG
#define debug(fmt, args...)   printf(fmt, ##args)
#else
#define debug(fmt, args...)   
#endif

#define LP_FIFO_SIZE  16      // Physical data FIFO lengths in Radio

#define RADIO_IDLE       0x00
#define RADIO_RX         0x80
#define RADIO_TX         0x20
#define RADIO_SOP        SOFDET_IRQ
#define RADIO_DATA       RXB1_IRQ
#define RADIO_COMPLETE   RXC_IRQ
#define RADIO_ERROR      RXE_IRQ

#define RADIO_RX_ACK      0x80
#define RADIO_RX_MISS     0x40
#define RADIO_RX_OF       0x20
#define RADIO_RX_CRC0     0x10
#define RADIO_BAD_CRC     0x08
#define RADIO_DATCODE_LEN 0x04
#define RADIO_SDR         0x03
#define RADIO_DDR         0x02
#define RADIO_8DR         0x01
#define RADIO_GFSK        0x00

#define RADIO_SOP_EN      0x80
#define RADIO_SOP_LEN     0x40
#define RADIO_LEN_EN      0x20
#define RADIO_SOP_THRESH_MSK 0x1F

#define RADIO_XS          0x80
#define RADIO_LV          0x40
#define RADIO_TXE         0x01

#define RADIO_ABORT_SUCCESS 0xFF

// Channel register
#define CHANNEL_ADR                                       0x00
#define CHANNEL_RST                                       0x48
#define CHANNEL_MSK                                       0x7F

#define CHANNEL_MAX                                       0x62
#define CHANNEL_MIN                                       0x00
#define CHANNEL_2P498_GHZ                                 0x62
#define CHANNEL_2P4_GHZ                                   0x00


// TX Length register
#define TX_LENGTH_ADR                                     0x01
#define TX_LENGTH_RST                                     0x00
#define TX_LENGTH_MSK                                     0xFF


// TX Control register
#define TX_CTRL_ADR                                       0x02
#define TX_CTRL_RST                                       0x03

// See TX_IRQ for remaining bit position definitions

// TX_CTRL bit masks
#define TX_GO                                             0x80
#define TX_CLR                                            0x40


// -------------------------------
// TX Configuration register
// -------------------------------
#define TX_CFG_ADR                                        0x03
#define TX_CFG_RST                                        0x05

// separate bit field masks
#define TX_DATCODE_LEN_MSK                                0x20
#define TX_DATMODE_MSK                                    0x18
#define PA_VAL_MSK                                        0x07

// DATCODE_LEN register masks
#define DATCODE_LEN_64                                    0x20
#define DATCODE_LEN_32                                    0x00

// DATMODE register masks
#define DATMODE_1MBPS                                     0x00
#define DATMODE_8DR                                       0x08
#define DATMODE_DDR                                       0x10
#define DATMODE_SDR                                       0x18

// PA_SET register masks
#define PA_N30_DBM                                        0x00
#define PA_N25_DBM                                        0x01
#define PA_N20_DBM                                        0x02
#define PA_N15_DBM                                        0x03
#define PA_N10_DBM                                        0x04
#define PA_N5_DBM                                         0x05
#define PA_0_DBM                                          0x06
#define PA_4_DBM                                          0x07


// TX IRQ Status register
#define TX_IRQ_STATUS_ADR                                 0x04

// TX_IRQ bit masks
#define XS_IRQ                                            0x80
#define LV_IRQ                                            0x40
#define TXB15_IRQ                                         0x20
#define TXB8_IRQ                                          0x10
#define TXB0_IRQ                                          0x08
#define TXBERR_IRQ                                        0x04
#define TXC_IRQ                                           0x02
#define TXE_IRQ                                           0x01


// RX Control register
#define RX_CTRL_ADR                                       0x05
#define RX_CTRL_RST                                       0x07

// See RX_IRQ register for bit positions definitions also used for this register

// RX_CTRL bit masks
#define RX_GO                                             0x80


// RX Configuration register
#define RX_CFG_ADR                                        0x06
#define RX_CFG_RST                                        0x92

#define AUTO_AGC_EN                                       0x80
#define LNA_EN                                            0x40
#define ATT_EN                                            0x20
#define HI                                                0x10
#define LO                                                0x00
#define FASTTURN_EN                                       0x08
#define RXOW_EN                                           0x02
#define VLD_EN                                            0x01


// RX IRQ register
#define RX_IRQ_STATUS_ADR                                 0x07

// RX_IRQ bit masks
#define RXOW_IRQ                                          0x80
#define SOFDET_IRQ                                        0x40
#define RXB16_IRQ                                         0x20
#define RXB8_IRQ                                          0x10
#define RXB1_IRQ                                          0x08
#define RXBERR_IRQ                                        0x04
#define RXC_IRQ                                           0x02
#define RXE_IRQ                                           0x01


// -------------------------------
// RX Status register
// -------------------------------
#define RX_STATUS_ADR                                     0x08

// single flag bits & multi-bit-field masks
#define RX_ACK                                            0x80
#define RX_PKTERR                                         0x40
#define RX_EOPERR                                         0x20
#define RX_CRC0                                           0x10
#define RX_BAD_CRC                                        0x08
#define RX_DATCODE_LEN                                    0x04
#define RX_DATMODE_MSK                                    0x03


// RX Count register
#define RX_COUNT_ADR                                      0x09
#define RX_COUNT_RST                                      0x00
#define RX_COUNT_MSK                                      0xFF


// RX Length Field register
#define RX_LENGTH_ADR                                     0x0A
#define RX_LENGTH_RST                                     0x00
#define RX_LENGTH_MSK                                     0xFF


// Power Control register
#define PWR_CTRL_ADR                                      0x0B
#define PWR_CTRL_RST                                      0xA0

// single flag bits & multi-bit-field masks
#define PMU_EN                                            0x80
#define LV_IRQ_EN                                         0x40
#define PMU_SEN                                           0x20
#define PFET_OFF                                          0x10
#define LV_IRQ_TH_MSK                                     0x0C
#define PMU_OUTV_MSK                                      0x03

// LV_IRQ_TH values
#define LV_IRQ_TH_1P8_V                                   0x0C
#define LV_IRQ_TH_2P0_V                                   0x08
#define LV_IRQ_TH_2P2_V                                   0x04
#define LV_IRQ_TH_PMU_OUTV                                0x00

// PMU_OUTV values
#define PMU_OUTV_2P4                                      0x03
#define PMU_OUTV_2P5                                      0x02
#define PMU_OUTV_2P6                                      0x01
#define PMU_OUTV_2P7                                      0x00


// Crystal Control register
#define XTAL_CTRL_ADR                                     0x0C
#define XTAL_CTRL_RST                                     0x04

// single flag bits & multi-bit-field masks
#define XOUT_FNC_MSK                                      0xC0
#define XS_IRQ_EN                                         0x20
#define XOUT_FREQ_MSK                                     0x07

// XOUT_FNC values
#define XOUT_FNC_XOUT_FREQ                                0x00
#define XOUT_FNC_PA_N                                     0x40
#define XOUT_FNC_RAD_STREAM                               0x80
#define XOUT_FNC_GPIO                                     0xC0

// XOUT_FREQ values
#define XOUT_FREQ_12MHZ                                   0x00
#define XOUT_FREQ_6MHZ                                    0x01
#define XOUT_FREQ_3MHZ                                    0x02
#define XOUT_FREQ_1P5MHZ                                  0x03
#define XOUT_FREQ_P75MHZ                                  0x04


// I/O Configuration register
#define IO_CFG_ADR                                        0x0D
#define IO_CFG_RST                                        0x00
#define IO_CFG_MSK                                        0xFF

// single flag bits & multi-bit-field masks
#define IRQ_OD                                            0x80
#define IRQ_POL                                           0x40
#define MISO_OD                                           0x20
#define XOUT_OD                                           0x10
#define PACTL_OD                                          0x08
#define PACTL_GPIO                                        0x04
#define SPI_3_PIN                                         0x02
#define IRQ_GPIO                                          0x01


// GPIO Control register
#define GPIO_CTRL_ADR                                     0x0E
#define GPIO_CTRL_RST                                     0x00
#define GPIO_CTRL_MSK                                     0xF0

// single flag bits & multi-bit-field masks
#define XOUT_OP                                           0x80
#define MISO_OP                                           0x40
#define PACTL_OP                                          0x20
#define IRQ_OP                                            0x10
#define XOUT_IP                                           0x08
#define MISO_IP                                           0x04
#define PACTL_IP                                          0x02
#define IRQ_IP                                            0x01


// Transaction Configuration register
#define XACT_CFG_ADR                                      0x0F
#define XACT_CFG_RST                                      0x80

// single flag bits & multi-bit-field masks
#define ACK_EN                                            0x80
#define FRC_END_STATE                                     0x20
#define END_STATE_MSK                                     0x1C
#define ACK_TO_MSK                                        0x03

// END_STATE field values
#define END_STATE_SLEEP                                   0x00
#define END_STATE_IDLE                                    0x04
#define END_STATE_TXSYNTH                                 0x08
#define END_STATE_RXSYNTH                                 0x0C
#define END_STATE_RX                                      0x10

// ACK_TO field values
#define ACK_TO_4X                                         0x00
#define ACK_TO_8X                                         0x01
#define ACK_TO_12X                                        0x02
#define ACK_TO_15X                                        0x03


// Framing Configuration register
#define FRAMING_CFG_ADR                                   0x10
#define FRAMING_CFG_RST                                   0xA5

// single flag bits & multi-bit-field masks
#define SOP_EN                                            0x80
#define SOP_LEN                                           0x40
#define LEN_EN                                            0x20
#define SOP_THRESH_MSK                                    0x1F


// Data Threshold 32 register
#define DATA32_THOLD_ADR                                  0x11
#define DAT32_THRESH_RST                                  0x04
#define DAT32_THRESH_MSK                                  0x0F


// Data Threshold 64 register
#define DATA64_THOLD_ADR                                  0x12
#define DAT64_THRESH_RST                                  0x0A
#define DAT64_THRESH_MSK                                  0x1F


// RSSI register
#define RSSI_ADR                                          0x13
#define RSSI_RST                                          0x20

// single flag bits & multi-bit-field masks
#define SOP_RSSI                                          0x80
#define LNA_STATE                                         0x20
#define RSSI_LVL_MSK                                      0x1F


// EOP Control register
#define EOP_CTRL_ADR                                      0x14
#define EOP_CTRL_RST                                      0xA4

// single flag bits & multi-bit-field masks
#define HINT_EN                                           0x80
#define HINT_EOP_MSK                                      0x70
#define EOP_MSK                                           0x0F


// CRC Seed registers
#define CRC_SEED_LSB_ADR                                  0x15
#define CRC_SEED_MSB_ADR                                  0x16
#define CRC_SEED_LSB_RST                                  0x00
#define CRC_SEED_MSB_RST                                  0x00

// CRC related values
// USB CRC-16
#define CRC_POLY_MSB                                      0x80
#define CRC_POLY_LSB                                      0x05
#define CRC_RESI_MSB                                      0x80
#define CRC_RESI_LSB                                      0x0D


// TX CRC Calculated registers
#define TX_CRC_LSB_ADR                                    0x17
#define TX_CRC_MSB_ADR                                    0x18


// RX CRC Field registers
#define RX_CRC_LSB_ADR                                    0x19
#define RX_CRC_MSB_ADR                                    0x1A
#define RX_CRC_LSB_RST                                    0xFF
#define RX_CRC_MSB_RST                                    0xFF


// Synth Offset registers
#define TX_OFFSET_LSB_ADR                                 0x1B
#define TX_OFFSET_MSB_ADR                                 0x1C
#define TX_OFFSET_LSB_RST                                 0x00
#define TX_OFFSET_MSB_RST                                 0x00

// single flag bits & multi-bit-field masks
#define STRIM_MSB_MSK                                     0x0F
#define STRIM_LSB_MSK                                     0xFF


// Mode Override register
#define MODE_OVERRIDE_ADR                                 0x1D
#define MODE_OVERRIDE_RST                                 0x00

#define FRC_AWAKE                                         0x03
#define FRC_AWAKE_OFF_1                                   0x01
#define FRC_AWAKE_OFF_2                                   0x00

// single flag bits & multi-bit-field masks
#define DIS_AUTO_SEN                                      0x80
#define SEN_TXRXB                                         0x40
#define FRC_SEN                                           0x20
#define FRC_AWAKE_MSK                                     0x18
#define MODE_OVRD_FRC_AWAKE                               0x18
#define MODE_OVRD_FRC_AWAKE_OFF_1                         0x08
#define MODE_OVRD_FRC_AWAKE_OFF_2                         0x00
#define RST                                               0x01
#define FRC_PA                                            0x02


// RX Override register
#define RX_OVERRIDE_ADR                                   0x1E
#define RX_OVERRIDE_RST                                   0x00

// single flag bits & multi-bit-field masks
#define ACK_RX                                            0x80
#define EXTEND_RX_TX                                      0x40
#define MAN_RXACK                                         0x20
#define FRC_RXDR                                          0x10
#define DIS_CRC0                                          0x08
#define DIS_RXCRC                                         0x04
#define ACE                                               0x02


// TX Override register
#define TX_OVERRIDE_ADR                                   0x1F
#define TX_OVERRIDE_RST                                   0x00

// single flag bits & multi-bit-field masks
#define ACK_TX_SEN                                        0x80
#define FRC_PREAMBLE                                      0x40
#define DIS_TX_RETRANS                                    0x20
#define MAN_TXACK                                         0x10
#define OVRRD_ACK                                         0x08
#define DIS_TXCRC                                         0x04
#define CO                                                0x02
#define TXINV                                             0x01


// TX Buffer FIFO - 16 bytes
#define TX_BUFFER_ADR                                     0x20


// RX Buffer FIFO - 16 bytes
#define RX_BUFFER_ADR                                     0x21


// Framing Code - 8 bytes
#define SOP_CODE_ADR                                      0x22

// CODESTORE_REG_SOF_RST        64'h17_ff_9e_21_36_90_c7_82
#define CODESTORE_BYTE7_SOF_RST                           0x17
#define CODESTORE_BYTE6_SOF_RST                           0xFF
#define CODESTORE_BYTE5_SOF_RST                           0x9E
#define CODESTORE_BYTE4_SOF_RST                           0x21
#define CODESTORE_BYTE3_SOF_RST                           0x36
#define CODESTORE_BYTE2_SOF_RST                           0x90
#define CODESTORE_BYTE1_SOF_RST                           0xC7
#define CODESTORE_BYTE0_SOF_RST                           0x82


// Data Code - 16 bytes
#define DATA_CODE_ADR                                     0x23

// CODESTORE_REG_DCODE0_RST            64'h01_2B_F1_DB_01_32_BE_6F
#define CODESTORE_BYTE7_DCODE0_RST                        0x01
#define CODESTORE_BYTE6_DCODE0_RST                        0x2B
#define CODESTORE_BYTE5_DCODE0_RST                        0xF1
#define CODESTORE_BYTE4_DCODE0_RST                        0xDB
#define CODESTORE_BYTE3_DCODE0_RST                        0x01
#define CODESTORE_BYTE2_DCODE0_RST                        0x32
#define CODESTORE_BYTE1_DCODE0_RST                        0xBE
#define CODESTORE_BYTE0_DCODE0_RST                        0x6F

// CODESTORE_REG_DCODE1_RST            64'h02_F9_93_97_02_FA_5C_E3
#define CODESTORE_BYTE7_DCODE1_RST                        0x02
#define CODESTORE_BYTE6_DCODE1_RST                        0xF9
#define CODESTORE_BYTE5_DCODE1_RST                        0x93
#define CODESTORE_BYTE4_DCODE1_RST                        0x97
#define CODESTORE_BYTE3_DCODE1_RST                        0x02
#define CODESTORE_BYTE2_DCODE1_RST                        0xFA
#define CODESTORE_BYTE1_DCODE1_RST                        0x5C
#define CODESTORE_BYTE0_DCODE1_RST                        0xE3


// Preamble - 3 bytes
#define PREAMBLE_ADR                                      0x24

#define PREAMBLE_CODE_MSB_RST                             0x33
#define PREAMBLE_CODE_LSB_RST                             0x33
#define PREAMBLE_LEN_RST                                  0x02


// Laser Fuses - 8 bytes (2 hidden)
#define MFG_ID_ADR                                        0x25


// XTAL Startup Delay
#define XTAL_CFG_ADR                                      0x26
#define XTAL_CFG_RST                                      0x00

// Clock Override
#define CLK_OVERRIDE_ADR                                  0x27
#define CLK_OVERRIDE_RST                                  0x00

#define RXF                                               0x02


// Clock Enable
#define CLK_EN_ADR                                        0x28
#define CLK_EN_RST                                        0x00

#define RXF                                               0x02


// Receiver Abort
#define RX_ABORT_ADR                                      0x29
#define RX_ABORT_RST                                      0x00

#define ABORT_EN                                          0x20


// Auto Calibration Time
#define AUTO_CAL_TIME_ADR                                 0x32
#define AUTO_CAL_TIME_RST                                 0x0C

#define AUTO_CAL_TIME_MAX                                 0x3C


// Auto Calibration Offset
#define AUTO_CAL_OFFSET_ADR                               0x35
#define AUTO_CAL_OFFSET_RST                               0x00

#define AUTO_CAL_OFFSET_MINUS_4                           0x14


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
  radio configuration data
 */
const AP_Radio_cypress::config AP_Radio_cypress::radio_config[] = {
    // reset
    { MODE_OVERRIDE_ADR, RST },

    { IO_CFG_ADR, IRQ_POL },

    // Force RXF clock on for streaming
    { CLK_OVERRIDE_ADR, RXF },
    { CLK_EN_ADR, RXF },

    // Use auto-cal for VCO. 
    { AUTO_CAL_TIME_ADR, AUTO_CAL_TIME_MAX },
    
    // AutoCal offset to -4.
    { AUTO_CAL_OFFSET_ADR, AUTO_CAL_OFFSET_MINUS_4 },
    
    // Set XTAL Startup delay to 150uSec to handle warm restarts of the XTAL. 
    { XTAL_CFG_ADR, 0x08 },

    // Enable HiLo for quick-turn-around. Use low side injection for receive - 
    // this should result in non-inverted data, so no need to hit the invert 
    // bit. Turn off AGC and force the LNA on. Enable receive overwrite
    { RX_CFG_ADR, (RX_CFG_RST | FASTTURN_EN | LNA_EN) & ~( HI | AUTO_AGC_EN ) },

    // Set the TX Offset to +1MHz.
    // THIS MEANS THE ACTUAL TRANSMIT CARRIER WILL BE 1MHz ABOVE THE PLL
    // FREQUENCY PROGRAMMED IN THE CHANNEL OR A & N REGISTERS.
    { TX_OFFSET_LSB_ADR, 0x55 },
    { TX_OFFSET_MSB_ADR, 0x05 },

    { XACT_CFG_ADR, 0x01 },
    { TX_CFG_ADR, 0x2F },

    { DATA64_THOLD_ADR, 0x0E },
    { DATA32_THOLD_ADR, 5 },

    { FRAMING_CFG_ADR, 0xEE },
};


// use PN Code 4 
const uint8_t AP_Radio_cypress::PnCode[] = {0x5C, 0xE1, 0xF6, 0x44, 0xAD, 0x16, 0xF6, 0x44};


/*
  read radio status, handling the race condition between completion and error
 */
uint8_t AP_Radio_cypress::read_status_debounced(uint8_t adr)
{
    uint8_t ret;

    dev->set_chip_select(true);
    ret = read_register(adr);

    // If COMPLETE and ERROR bits mismatch, then re-read register
    if ((ret & (RXC_IRQ | RXE_IRQ)) != 0 
        && (ret & (RXC_IRQ | RXE_IRQ)) != (RXC_IRQ | RXE_IRQ)) {
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
        write_register(XACT_CFG_ADR, FRC_END_STATE);
        uint32_t start_ms = AP_HAL::millis();
        do {
            if ((read_register(XACT_CFG_ADR) & FRC_END_STATE) == 0) {
                return;                     // FORCE_END done (osc running)
            }
        } while (AP_HAL::millis() - start_ms < 5);
        
        // FORCE_END failed to complete, implying going SLEEP to IDLE and 
        // oscillator failed to start.  Recover by returning to SLEEP and
        //  trying to start oscillator again.
        write_register(XACT_CFG_ADR, END_STATE_SLEEP);
    }
}

/*
  set desired channel
 */
void AP_Radio_cypress::set_channel(uint8_t channel)
{
    uint8_t frequency = channel + 1;
    write_register(CHANNEL_ADR, frequency);
}

/*
  initialise the radio
 */
void AP_Radio_cypress::radio_init(void)
{
    printf("radio_init starting\n");

    // wait for radio to settle
    while (true) {
        uint8_t chan = read_register(CHANNEL_ADR);
        if (chan == 1) {
            break;
        }
        write_register(CHANNEL_ADR, 1);
        hal.scheduler->delay(10);
    }
    
    // setup required radio config
    for (uint8_t i=0; i<ARRAY_SIZE(radio_config); i++) {
        write_register(radio_config[i].reg, radio_config[i].value);
    }
    set_channel(0);

    // set the Pn code
    write_multiple(SOP_CODE_ADR, sizeof(PnCode), PnCode);

    write_register(XTAL_CTRL_ADR,0x80);  // XOUT=BitSerial
    force_initial_state();
    write_register(PWR_CTRL_ADR,0x20);   // Disable PMU
    debug("radio_init done\n");

    // setup handler for rising edge of IRQ pin
    //stm32_gpiosetevent(GPIO_GPIO4_INPUT, true, false, false, irq_trampoline);
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

    sem_init(&state.sem, 0, 0);

    uint8_t n = MIN(length, 16);
    state.data = data + n;
    state.length = length - n;
    state.status = OP_IN_PROGRESS;

    // pre-fill TX FIFO with up to 16 bytes
    write_register(TX_LENGTH_ADR, length);
    write_register(TX_CTRL_ADR, TX_CLR);

    if (n > 0) {
        write_multiple(TX_BUFFER_ADR, n, data);
    }

    debug("starting send of %u sent n=%u\n", length, n);
    
    uint8_t irq_bits = TX_GO | TXC_IRQ | TXE_IRQ | TXBERR_IRQ;
    if (state.length > 0) {
        // ask for an interrupt when we can fit more bytes into FIFO
        irq_bits |= TXB0_IRQ | TXB8_IRQ | TXB15_IRQ;
    }

    state.start_us = AP_HAL::micros();
    debug("setting up tx_state at %u\n", state.start_us);
    hal.scheduler->delay(10);

    write_register(TX_CTRL_ADR, irq_bits);

    tx_state = &state;

    while (true) {
        uint8_t tx_status = read_status_debounced(TX_IRQ_STATUS_ADR);
        if (tx_status & (TXE_IRQ | TXBERR_IRQ | TXC_IRQ | TXB0_IRQ | TXB8_IRQ | TXB15_IRQ)) {
            //debug("tx_status=0x%02x at %u\n", (unsigned)tx_status, AP_HAL::micros() - tx_state->start_us);
            if (tx_status & (TXE_IRQ | TXBERR_IRQ)) {
                // got an error
                write_register(TX_CTRL_ADR, TX_CLR);
                debug("tx error tx_status=0x%02x at %u\n", (unsigned)tx_status, AP_HAL::micros() - tx_state->start_us);
                tx_state->status = OP_ERROR;
                goto done;
            }

            if (tx_status & TXC_IRQ) {
                write_register(TX_CTRL_ADR, TX_CLR);
#if 1
                debug("tx DONE tx_status=0x%02x rem=%u at %u\n",
                       (unsigned)tx_status, tx_state->length, AP_HAL::micros() - tx_state->start_us);
#endif
                tx_state->status = OP_OK;
                goto done;
            }
            
            if (tx_status & (TXB0_IRQ|TXB8_IRQ|TXB15_IRQ)) {
                uint8_t space;
                if (tx_status & TXB0_IRQ) {
                    space = 16;
                } else if (tx_status & TXB8_IRQ) {
                    space = 8;
                } else {
                    space = 1;
                }
                n = MIN(tx_state->length, space);
                
                if (n == 0) {
                    // clear interrupt bits to stop us getting interrupted
                    write_register(TX_CTRL_ADR, TXC_IRQ | TXE_IRQ | TXBERR_IRQ);
                } else {
                    write_multiple(TX_BUFFER_ADR, n, tx_state->data);
                    
                    //debug("added %u bytes at %u\n", n, AP_HAL::micros() - tx_state->start_us);
                    tx_state->length -= n;
                    tx_state->data += n;
                    
                    if (tx_state->length == 0) {
                        write_register(TX_CTRL_ADR, TXC_IRQ | TXE_IRQ | TXBERR_IRQ);
                    }
                }
            }
        }
    }

done:
    debug("TX finished at %u\n", AP_HAL::micros() - state.start_us);
    
    dev->get_semaphore()->give();
    tx_state = nullptr;

    return state.status == OP_OK;
}

/*
  handle radio interrupt
 */
void AP_Radio_cypress::irq_handler(void)
{
    uint8_t tx_status = read_status_debounced(TX_IRQ_STATUS_ADR);
    (void)read_status_debounced(RX_IRQ_STATUS_ADR);
    if (tx_state == nullptr) {
        //debug("NULL tx_status=0x%02x at %u\n", (unsigned)tx_status);
        return;
    }
    while (tx_status & (TXE_IRQ | TXBERR_IRQ | TXC_IRQ | TXB0_IRQ | TXB8_IRQ | TXB15_IRQ)) {
        //debug("tx_status=0x%02x at %u\n", (unsigned)tx_status, AP_HAL::micros() - tx_state->start_us);
        if (tx_status & (TXE_IRQ | TXBERR_IRQ)) {
            // got an error
            write_register(TX_CTRL_ADR, TX_CLR);
            debug("tx error tx_status=0x%02x at %u\n", (unsigned)tx_status, AP_HAL::micros() - tx_state->start_us);
            tx_state->status = OP_ERROR;
            sem_post(&tx_state->sem);
            return;
        }

        if (tx_status & TXC_IRQ) {
            write_register(TX_CTRL_ADR, TX_CLR);
#if 1
            debug("tx DONE tx_status=0x%02x rem=%u at %u\n",
                   (unsigned)tx_status, tx_state->length, AP_HAL::micros() - tx_state->start_us);
#endif
            tx_state->status = OP_OK;
            sem_post(&tx_state->sem);
            return;
        }

        if (tx_status & (TXB0_IRQ|TXB8_IRQ|TXB15_IRQ)) {
            uint8_t space;
            if (tx_status & TXB0_IRQ) {
                space = 16;
            } else if (tx_status & TXB8_IRQ) {
                space = 8;
            } else {
                space = 1;
            }
            uint8_t n = MIN(tx_state->length, space);

            if (n == 0) {
                // clear interrupt bits to stop us getting interrupted
                write_register(TX_CTRL_ADR, TXC_IRQ | TXE_IRQ | TXBERR_IRQ);
            } else {
                write_multiple(TX_BUFFER_ADR, n, tx_state->data);

                //debug("added %u bytes at %u\n", n, AP_HAL::micros() - tx_state->start_us);
                tx_state->length -= n;
                tx_state->data += n;

                if (tx_state->length == 0) {
                    write_register(TX_CTRL_ADR, TXC_IRQ | TXE_IRQ | TXBERR_IRQ);
                }
            }
        }
        tx_status = read_status_debounced(TX_IRQ_STATUS_ADR);
    }
}

uint8_t AP_Radio_cypress::streaming_receive(uint8_t *data, uint8_t maxlen)
{
    uint8_t *data0 = data;
    uint8_t maxlen0 = maxlen;
    write_register(RX_CTRL_ADR, RX_GO);
    uint8_t rx_status;
    uint8_t ret = 0;
    while (true) {
        rx_status = read_status_debounced(RX_IRQ_STATUS_ADR);
        if (rx_status == 0) {
            continue;
        }
        //debug("rx_status 0x%02x\n", rx_status);
        if (rx_status & RXOW_IRQ) {
            write_register(RX_IRQ_STATUS_ADR, RXOW_IRQ);
        }
        if ((rx_status & (RXB16_IRQ | RXB8_IRQ | RXB1_IRQ)) && maxlen > 16) {
            uint8_t avail;
            if (rx_status & RXB16_IRQ) {
                avail = 16;
            } else if (rx_status & RXB8_IRQ) {
                avail = 8;
            } else {
                avail = 1;
            }
            uint8_t n = MIN(maxlen, avail);
            if (n > 0) {
                dev->read_registers(RX_BUFFER_ADR, data, n);
                data += n;
                maxlen -= n;
                ret += n;
            }
        }
        if (rx_status & RXC_IRQ) {
            break;
        }
        if (rx_status & (RXBERR_IRQ | RXE_IRQ)) {
            debug("rx_status=0x%02x clearing %u bytes\n", rx_status, ret);
            data = data0;
            maxlen = maxlen0;
            ret = 0;
            write_register(RX_CTRL_ADR, RX_GO);
            continue;
        }
    }
    uint8_t rlen = read_register(RX_LENGTH_ADR);
#if RADIO_DEBUG
    uint8_t rem = read_register(RX_COUNT_ADR);
    uint8_t ret0 = ret;
#endif
    
    uint8_t n = rlen - ret;
    if (n > maxlen) {
        n = maxlen;
    }
    if (n > 0) {
        dev->read_registers(RX_BUFFER_ADR, data, n);
        ret += n;
    }

    // Write desired End State using FORCE_END_STATE feature
    do {
        write_register(XACT_CFG_ADR, FRC_END_STATE);
        // Wait for FORCE_END to complete
        hal.scheduler->delay_microseconds(100);
    } while ((read_register(XACT_CFG_ADR) & FRC_END_STATE) != 0);
    
    debug("receive ret=%u ret0=%u rlen=%u rem=%u rx_status=0x%02x\n", ret, ret0, rlen, rem, rx_status);
    return ret;
}

/*
  called on rising edge of radio IRQ pin
 */
int AP_Radio_cypress::irq_trampoline(int irq, void *context)
{
    radio_instance->irq_handler();
    return 0;
}

