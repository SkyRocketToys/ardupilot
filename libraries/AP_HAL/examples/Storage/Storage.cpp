/*
  simple test of Storage API
 */

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class Storage : public AP_HAL::HAL::Callbacks {
public:
    void setup() override;
    void loop() override;
private:
    uint32_t offset=0;
    uint32_t count=0;
};
    
void Storage::setup(void) 
{
    AP_HAL::Storage *st = hal.storage;
    hal.console->begin(115200);
    hal.console->printf("Starting AP_HAL::Storage test\n");
    st->init();
}

// In main loop do nothing
void Storage::loop(void) 
{
    if (count % 4096 == 0) {
        hal.console->printf("write %u at %u\n", (unsigned)count, (unsigned)offset);
    }
    for (uint32_t i=0; i<256; i++) {
        hal.storage->write_block(offset, &count, sizeof(count));
        offset = (offset + 4) % HAL_STORAGE_SIZE;
        count++;
    }
    hal.scheduler->delay(10);
}

Storage storage;

AP_HAL_MAIN_CALLBACKS(&storage);
