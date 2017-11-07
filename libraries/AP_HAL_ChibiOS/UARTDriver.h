#pragma once

#include <AP_HAL/utility/RingBuffer.h>

#include "AP_HAL_ChibiOS.h"

#define RX_BOUNCE_BUFSIZE 32
class ChibiOS::ChibiUARTDriver : public AP_HAL::UARTDriver {
public:
    ChibiUARTDriver(uint8_t serial_num);

    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();


    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;
    void _timer_tick(void);


    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);
    struct SerialDef {
        BaseSequentialStream* serial;
        bool is_usb;
        bool dma_rx;
        uint8_t dma_stream_id;
        uint8_t dma_channel_id; 
    };
private:
    bool _dma_rx;
    uint8_t _serial_num;
    uint32_t _baudrate;
    BaseSequentialStream* _serial;
    SerialConfig sercfg;
    bool _is_usb;
    const thread_t* _uart_owner_thd;
    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
    uint8_t rx_bounce_buf[RX_BOUNCE_BUFSIZE];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    mutex_t _write_mutex;
    const stm32_dma_stream_t* rxdma;
    bool _in_timer;
    bool _nonblocking_writes;
    bool _initialised;
    static void rx_irq_cb(void* sd);
    static void rxbuff_full_irq(void* self, uint32_t flags);
};
