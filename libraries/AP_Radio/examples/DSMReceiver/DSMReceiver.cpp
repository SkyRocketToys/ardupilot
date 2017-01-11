/*
 *  DSM receiver example code for Cypress radio
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Radio/AP_Radio.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Radio radio;

void setup()
{
    hal.console->printf("RADIO init\n");
    hal.scheduler->delay(1000);
    radio.init();
}

#if 1
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
#endif

extern uint8_t rf_chan;

void loop()
{
    static uint8_t chan;
    static uint32_t counter;
    static uint32_t last_print_us;
    static uint32_t last_pkt_us;
    static uint8_t lost_count;
    static bool next_packet_fast;
    uint8_t pkt[16];
    
    uint8_t len = radio.recv(pkt, sizeof(pkt), next_packet_fast?5000:8000);
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
        hal.console->printf("len=%u size=%u\n", len, sizeof(pkt));
    } else {
        lost_count = 0;
#if 1
        hal.console->printf("PKT[%02X,%02X]: ", chan, rf_chan);
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
#endif
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

    if (lost_count < 3) {
        radio.next_channel();
        chan = (chan+1) % 23;
    }
}

AP_HAL_MAIN();
