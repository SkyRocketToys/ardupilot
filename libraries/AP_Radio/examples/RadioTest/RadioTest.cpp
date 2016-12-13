/*
 *  Example code for AP_Radio
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Radio/AP_Radio.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Radio radio;

void setup()
{
    printf("RADIO init\n");
    hal.scheduler->delay(1000);
    radio.init();
}

static const uint8_t num_pad = 8;

struct PACKED test_packet {
    uint32_t seq;
    uint32_t not_seq;
    uint32_t pad[num_pad];
};

bool packet_ok(const struct test_packet &pkt)
{
    if (pkt.seq != ~pkt.not_seq) {
        return false;
    }
    for (uint8_t i=0; i<num_pad; i++) {
        if (pkt.pad[i] != pkt.seq + 100*(i+1)) {
            return false;
        }
    }
    return true;
}

void packet_create(struct test_packet &pkt, uint32_t seq)
{
    pkt.seq = seq;
    pkt.not_seq = ~seq;
    for (uint8_t i=0; i<num_pad; i++) {
        pkt.pad[i] = pkt.seq + 100*(i+1);
    }
}

void loop()
{
    static struct test_packet pkt;
    static uint32_t counter;
    static uint32_t last_print_ms;
#if 0
    static uint32_t recv_fail_count;
    static uint32_t corrupt_count;
    static uint32_t good_count;
    static uint32_t lost_count;
    static uint32_t last_seq;
    uint8_t len = radio.recv((uint8_t *)&pkt, sizeof(pkt));
    if (len != sizeof(pkt)) {
        printf("len=%u size=%u\n", len, sizeof(pkt));
        recv_fail_count++;
    } else if (!packet_ok(pkt)) {
        corrupt_count++;
        printf("corrupt seq=0x%08x !seq=0x%08x\n", pkt.seq, pkt.not_seq);
    } else if (pkt.seq == 0) {
        // sender reset
        printf("good reset\n");
        counter++;
        good_count = 1;
        lost_count = 0;
        last_seq = 0;
    } else {
        //printf("good seq=0x%08x\n", pkt.seq);
        counter++;
        good_count++;
        lost_count += (pkt.seq - last_seq) - 1;
        last_seq = pkt.seq;
    }
    uint32_t now = AP_HAL::millis();
    if (now - last_print_ms > 1000) {
        last_print_ms = now;
        printf("pps:%u recv_fail:%u corrupt:%u good:%u lost:%u seq:%u\n",
               counter, recv_fail_count, corrupt_count, good_count, lost_count, pkt.seq);
        counter = 0;
    }
    if (recv_fail_count > 1000) {
        printf("Resetting radio\n");
        radio.reset();
        recv_fail_count = 0;
    }
#else
    static uint32_t seq;
    static uint32_t send_err;
    packet_create(pkt, seq);
    seq++;
    counter++;
    if (!radio.send((const uint8_t *)&pkt, sizeof(pkt))) {
        send_err++;
    }
    uint32_t now = AP_HAL::millis();
    if (now - last_print_ms > 1000) {
        last_print_ms = now;
        printf("%u pps send seq=%u err=%u\n", counter, seq, send_err);
        counter=0;
    }
    hal.scheduler->delay_microseconds(250);
#endif
}

AP_HAL_MAIN();
