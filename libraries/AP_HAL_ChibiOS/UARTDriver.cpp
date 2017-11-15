#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "UARTDriver.h"
#include "GPIO.h"
#include <usbcfg.h>

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_PIXHAWK_CUBE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_V2450
#define HAVE_USB_SERIAL
#endif

static ChibiUARTDriver::SerialDef _serial_tab[] = {
    {(BaseSequentialStream*) &SD2, false, true,
      STM32_UART_USART2_RX_DMA_STREAM, 
      STM32_DMA_GETCHANNEL(STM32_UART_USART2_RX_DMA_STREAM, STM32_USART2_RX_DMA_CHN)
    },   //Serial 0
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_PIXHAWK_CUBE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_V2450
    {(BaseSequentialStream*) &SD4, false, false, 0, 0},   //Serial 1
    {(BaseSequentialStream*) &SDU1, true, false, 0, 0},   //Serial 2
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
    {(BaseSequentialStream*) &SD6, false, false, 0, 0},   //Serial 1
#endif
};

ChibiUARTDriver::ChibiUARTDriver(uint8_t serial_num) :
_serial_num(serial_num),
_baudrate(57600),
_is_usb(false),
_in_timer(false),
_initialised(false)
{
    _serial = _serial_tab[serial_num].serial;
    _is_usb = _serial_tab[serial_num].is_usb;
    _dma_rx = _serial_tab[serial_num].dma_rx;
    chMtxObjectInit(&_write_mutex);
}

void ChibiUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_serial == nullptr) {
        return;
    }
    bool was_initialised = _initialised;
    uint16_t min_tx_buffer = 4096;
    uint16_t min_rx_buffer = 1024;
    // on PX4 we have enough memory to have a larger transmit and
    // receive buffer for all ports. This means we don't get delays
    // while waiting to write GPS config packets
    if (txS < min_tx_buffer) {
        txS = min_tx_buffer;
    }
    if (rxS < min_rx_buffer) {
        rxS = min_rx_buffer;
    }

    /*
      allocate the read buffer
      we allocate buffers before we successfully open the device as we
      want to allocate in the early stages of boot, and cause minimum
      thrashing of the heap once we are up. The ttyACM0 driver may not
      connect for some time after boot
     */
    if (rxS != _readbuf.get_size()) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }

        _readbuf.set_size(rxS);
    }

    if (b != 0) {
        _baudrate = b;
    }

    /*
      allocate the write buffer
     */
    if (txS != _writebuf.get_size()) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }
        _writebuf.set_size(txS);
    }

    if (_is_usb) {
#ifdef HAVE_USB_SERIAL
        /*
         * Initializes a serial-over-USB CDC driver.
         */
        if (!was_initialised) {
            sduObjectInit((SerialUSBDriver*)_serial);
            sduStart((SerialUSBDriver*)_serial, &serusbcfg);
            /*
             * Activates the USB driver and then the USB bus pull-up on D+.
             * Note, a delay is inserted in order to not have to disconnect the cable
             * after a reset.
             */
            usbDisconnectBus(serusbcfg.usbp);
            hal.scheduler->delay_microseconds(1500);
            usbStart(serusbcfg.usbp, &usbcfg);
            usbConnectBus(serusbcfg.usbp);
        }
#endif
    } else {
        if (_baudrate != 0) {
            //setup Rx DMA
            if(!was_initialised && _dma_rx) {
                rxdma = STM32_DMA_STREAM(_serial_tab[_serial_num].dma_stream_id);
                bool b = dmaStreamAllocate(rxdma,
                                           0,
                                           (stm32_dmaisr_t)rxbuff_full_irq,
                                           (void *)this);
                osalDbgAssert(!b, "stream already allocated");
                dmaStreamSetPeripheral(rxdma, &((SerialDriver*)_serial)->usart->DR);
            }
            sercfg.speed = _baudrate;
            if (_dma_rx) {
                sercfg.cr1 = USART_CR1_IDLEIE;
                sercfg.cr3 = USART_CR3_DMAR;
            } else {
                sercfg.cr1 = 0;
                sercfg.cr3 = 0;
            }
            sercfg.cr2 = USART_CR2_STOP1_BITS;
            sercfg.irq_cb = rx_irq_cb;
            sercfg.ctx = (void*)this;
            
            sdStart((SerialDriver*)_serial, &sercfg);
            if(_dma_rx) {
                //Configure serial driver to skip handling RX packets
                //because we will handle them via DMA
                ((SerialDriver*)_serial)->usart->CR1 &= ~USART_CR1_RXNEIE;
                //Start DMA
                if(!was_initialised) {
                    uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | 
                                       STM32_DMA_CR_CHSEL(_serial_tab[_serial_num].dma_channel_id) |
                                       STM32_DMA_CR_PL(0);
                    dmaStreamSetMemory0(rxdma, rx_bounce_buf);
                    dmaStreamSetTransactionSize(rxdma, RX_BOUNCE_BUFSIZE);
                    dmaStreamSetMode(rxdma, dmamode    | STM32_DMA_CR_DIR_P2M |
                                         STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
                    dmaStreamEnable(rxdma);
                }
            }
        }
    }

    if (_writebuf.get_size() && _readbuf.get_size()) {
        _initialised = true;
    }
    _uart_owner_thd = chThdGetSelfX();
}

void ChibiUARTDriver::rx_irq_cb(void* self)
{
    ChibiUARTDriver* uart_drv = (ChibiUARTDriver*)self;
    if (!uart_drv->_dma_rx) {
        return;
    }
    volatile uint16_t sr = ((SerialDriver*)(uart_drv->_serial))->usart->SR;
    if(sr & USART_SR_IDLE) {
        volatile uint16_t dr = ((SerialDriver*)(uart_drv->_serial))->usart->DR;
        (void)dr;
        //disable dma, triggering DMA transfer complete interrupt
        uart_drv->rxdma->stream->CR &= ~STM32_DMA_CR_EN;
    }
}

void ChibiUARTDriver::rxbuff_full_irq(void* self, uint32_t flags)
{
    ChibiUARTDriver* uart_drv = (ChibiUARTDriver*)self;
    if (!uart_drv->_dma_rx) {
        return;
    }
    uint8_t len = RX_BOUNCE_BUFSIZE - uart_drv->rxdma->stream->NDTR;
    if (len == 0) {
        return;
    }
    uart_drv->_readbuf.write(uart_drv->rx_bounce_buf, len);
    //restart the DMA transfers
    dmaStreamSetMemory0(uart_drv->rxdma, uart_drv->rx_bounce_buf);
    dmaStreamSetTransactionSize(uart_drv->rxdma, RX_BOUNCE_BUFSIZE);
    dmaStreamEnable(uart_drv->rxdma);
}

void ChibiUARTDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void ChibiUARTDriver::end()
{
    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

    if (_is_usb) {
#ifdef HAVE_USB_SERIAL

        sduStop((SerialUSBDriver*)_serial);
#endif
    } else {
        sdStop((SerialDriver*)_serial);
    }
    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

void ChibiUARTDriver::flush()
{
    if (_is_usb) {
#ifdef HAVE_USB_SERIAL

        sduSOFHookI((SerialUSBDriver*)_serial);
#endif
    } else {
        //TODO: Handle this for other serial ports
    }
}

bool ChibiUARTDriver::is_initialized()
{
    return _initialised;
}

void ChibiUARTDriver::set_blocking_writes(bool blocking)
{
    _nonblocking_writes = !blocking;
}

bool ChibiUARTDriver::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
uint32_t ChibiUARTDriver::available() {
    if (!_initialised) {
        return 0;
    }
    if (_is_usb) {
#ifdef HAVE_USB_SERIAL

        if (((SerialUSBDriver*)_serial)->config->usbp->state != USB_ACTIVE) {
            return 0;
        }
#endif
    }
    return _readbuf.available();
}

uint32_t ChibiUARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }
    return _writebuf.space();
}

int16_t ChibiUARTDriver::read()
{
    if (_uart_owner_thd != chThdGetSelfX()){
        return -1;
    }
    if (!_initialised) {
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }

    return byte;
}

/* Empty implementations of Print virtual methods */
size_t ChibiUARTDriver::write(uint8_t c)
{
    if (!chMtxTryLock(&_write_mutex)) {
        return -1;
    }
    
    if (!_initialised) {
        chMtxUnlock(&_write_mutex);
        return 0;
    }

    while (_writebuf.space() == 0) {
        if (_nonblocking_writes) {
            chMtxUnlock(&_write_mutex);
            return 0;
        }
        hal.scheduler->delay(1);
    }
    size_t ret = _writebuf.write(&c, 1);
    chMtxUnlock(&_write_mutex);
    return ret;
}

size_t ChibiUARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
		return 0;
	}

    if (!chMtxTryLock(&_write_mutex)) {
        return -1;
    }

    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        chMtxUnlock(&_write_mutex);
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    size_t ret = _writebuf.write(buffer, size);
    chMtxUnlock(&_write_mutex);
    return ret;
}


/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void ChibiUARTDriver::_timer_tick(void)
{
    int ret;
    uint32_t n;

    if (!_initialised) return;

    // don't try IO on a disconnected USB port
    if (_is_usb) {
#ifdef HAVE_USB_SERIAL
        if (((SerialUSBDriver*)_serial)->config->usbp->state != USB_ACTIVE) {
            return;
        }
#endif
    }
    if(_is_usb) {
#ifdef HAVE_USB_SERIAL
        ((ChibiGPIO *)hal.gpio)->set_usb_connected();
#endif
    }
    _in_timer = true;

    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];

    const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        //Do a non-blocking read
        if (_is_usb) {
            ret = 0;
#ifdef HAVE_USB_SERIAL
            ret = chnReadTimeout((SerialUSBDriver*)_serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        } else if(!_dma_rx){
            ret = 0;
            ret = chnReadTimeout((SerialDriver*)_serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
        } else {
            ret = 0;
        }
        if (ret < 0) {
            break;
        }
        _readbuf.commit((unsigned)ret);

        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }

    // write any pending bytes
    n = _writebuf.available();
    if (n > 0) {
        ByteBuffer::IoVec vec[2];
        const auto n_vec = _writebuf.peekiovec(vec, n);
        for (int i = 0; i < n_vec; i++) {
            if (_is_usb) {
                ret = 0;
#ifdef HAVE_USB_SERIAL
                ret = chnWriteTimeout((SerialUSBDriver*)_serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
            } else {
                ret = chnWriteTimeout((SerialDriver*)_serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
            }
            if (ret < 0) {
                break;
            }
            _writebuf.advance(ret);

            /* We wrote less than we asked for, stop */
            if ((unsigned)ret != vec[i].len) {
                break;
            }
        }
    }

    _in_timer = false;
}
#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
