/*
 *  DSM receiver example code for Cypress radio
 *
 * With thanks to the SuperBitRF project and their excellent cyrf6936 dongle:
 *  https://1bitsquared.com/products/superbit-usbrf-dongle
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Radio/AP_Radio.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Radio radio;

static bool do_bind = true;

void setup()
{
    hal.console->printf("RADIO init\n");
    hal.scheduler->delay(1000);
    hal.uartA->begin(115200);
    radio.init();
    if (do_bind) {
        radio.start_bind();
    }
}

/**
 * Convert normal radio transmitter to channel outputs
 */
static void convert_radio_to_channels(uint8_t* data, uint8_t nb_channels, bool is_11bit, int16_t* channels)
{
	int i;
	uint8_t bit_shift = (is_11bit)? 11:10;
	int16_t value_max = (is_11bit)? 0x07FF: 0x03FF;

	for (i=0; i<7; i++) {
		const int16_t tmp = ((data[2*i]<<8) + data[2*i+1]) & 0x7FFF;
		const uint8_t chan = (tmp >> bit_shift) & 0x0F;
		const int16_t val  = (tmp&value_max);

		if(chan < nb_channels)
			channels[chan] = val;
	}

    hal.uartA->begin(115200);
}

extern uint8_t rf_chan;

static bool handle_bind(uint8_t pkt[16], uint8_t len)
{
    printf("PKT[%02x]: ", rf_chan);
    for (uint8_t i=0; i<len; i++) {
        printf("%02x ", pkt[i]);
    }
    // when binding the OrangeRX DSMX transmitter seems to send a packet every 10ms
    printf("t=%u\n", AP_HAL::micros());
    
    bool ok = (len==16 && pkt[0] == pkt[4] && pkt[1] == pkt[5] && pkt[2] == pkt[6] && pkt[3] == pkt[7]);

    // Calculate the first sum
    uint16_t bind_sum = 384 - 0x10;
    for (uint8_t i = 0; i < 8; i++) {
        bind_sum += pkt[i];
    }

    // Check the first sum
    if (pkt[8] != bind_sum >> 8 || pkt[9] != (bind_sum & 0xFF)) {
        printf("bind sum1 bad\n");
        ok = false;
    }

    // Calculate second sum
    for (uint8_t i = 8; i < 14; i++) {
        bind_sum += pkt[i];
    }

    // Check the second sum
    if (pkt[14] != bind_sum >> 8 || pkt[15] != (bind_sum & 0xFF)) {
        printf("bind sum2 bad\n");
        ok = false;
    }

    if (ok) {
        uint8_t mfg_id[4] = {uint8_t(~pkt[0]), uint8_t(~pkt[1]), uint8_t(~pkt[2]), uint8_t(~pkt[3])};
        uint8_t num_channels = pkt[11];
        uint8_t protocol = pkt[12];
        
        printf("BIND OK: mfg_id={0x%02x, 0x%02x, 0x%02x, 0x%02x} N=%u P=0x%02x\n",
               mfg_id[0], mfg_id[1], mfg_id[2], mfg_id[3],
               num_channels,
               protocol);
    }
    return ok;
}

void loop()
{
    static uint32_t counter;
    static uint32_t last_print_us;
    static uint32_t last_pkt_us;
    static uint8_t lost_count;
    static bool next_packet_fast;
    static uint32_t last_change_chan_us;
    static uint32_t last_ok_us;
    uint8_t pkt[16];
    uint32_t pkt_timeout;

    if (do_bind) {
        pkt_timeout = 15000;
    } else {
        pkt_timeout = next_packet_fast?5000:8000;
    }
    
    uint8_t len = radio.recv(pkt, sizeof(pkt), pkt_timeout);
    uint32_t now = AP_HAL::micros();

    uint32_t dt = now - last_pkt_us;

    if (dt < 4800) {
        // this packet was fast, next one won't be
        next_packet_fast = false;
    } else {
        next_packet_fast = !next_packet_fast;
    }
    
    if (len == 0) {
        lost_count++;
    } else if (len != sizeof(pkt)) {
        printf("len=%u size=%u\n", len, sizeof(pkt));
    } else {
        lost_count = 0;
        if (!do_bind) {
            hal.console->printf("PKT[%02X]: ", rf_chan);
            for (uint8_t i=0; i<2; i++) {
                hal.console->printf("%02x ", pkt[i]);
            }
            const uint8_t num_channels = 14;
            static int16_t channels[num_channels] {};
            convert_radio_to_channels(&pkt[2], num_channels, true, channels);
            for (uint8_t i=0; i<num_channels; i++) {
                hal.console->printf("%u:%4u ", i+1, channels[i]);
            }
            hal.console->printf("dt=%u", dt);
            hal.console->printf("\n");
        } else {
            if (handle_bind(pkt, len)) {
                last_ok_us = now;
            }
        }
        last_pkt_us = now;
        counter++;
    }
    if (now - last_print_us > 1000*1000U) {
        if (last_print_us != 0) {
            hal.console->printf("pps:%.1f counter=%u\n", (1.0e6 * counter) / (now - last_print_us), counter);
        }
        last_print_us = now;
        counter = 0;
    }

    if (do_bind) {
        if (now - last_change_chan_us >= 20000 && now - last_ok_us > 1000000U) {
            radio.next_channel();
            last_change_chan_us = now;
        }
    } else {
        if (lost_count < 3) {
            radio.next_channel();
        }
    }
}

AP_HAL_MAIN();
