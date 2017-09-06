/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>

namespace ChibiOS {

static SPIDriver* spi_devices[] = {
    &SPID1,
    &SPID2
};

#define MHZ (1000U*1000U)
#define KHZ (1000U)
SPIDesc SPIDeviceManager::device_table[] = {
    SPIDesc("bmp280", SPI_BUS_SENSORS, SPIDEV_BMP280, SPIDEV_CS_BMP280, SPIDEV_MODE3, 1*MHZ, 10*KHZ ),
    SPIDesc("lsm303d", SPI_BUS_SENSORS, SPIDEV_LSM303D, SPIDEV_CS_LSM303D, SPIDEV_MODE3, 11*MHZ, 11*KHZ ),
    SPIDesc("l3gd20h", SPI_BUS_SENSORS, SPIDEV_L3GD20H , SPIDEV_CS_L3GD20H, SPIDEV_MODE3, 11*MHZ, 11*KHZ )
};

SPIDevice::SPIDevice(SPIBus &_bus, SPIDesc &_device_desc)
    : bus(_bus)
    , device_desc(_device_desc)
{
    set_device_bus(_bus.bus);
    set_device_address(_device_desc.device);
    set_speed(AP_HAL::Device::SPEED_LOW);

    asprintf(&pname, "SPI:%s:%u:%u",
             device_desc.name,
             (unsigned)bus.bus, (unsigned)device_desc.device);
    //printf("SPI device %s on %u:%u at speed %u mode %u\n",
    //       device_desc.name,
    //       (unsigned)bus.bus, (unsigned)device_desc.device,
    //       (unsigned)frequency, (unsigned)device_desc.mode);
}

SPIDevice::~SPIDevice()
{
    //printf("SPI device %s on %u:%u closed\n", device_desc.name,
    //       (unsigned)bus.bus, (unsigned)device_desc.device);
    free(pname);
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
        frequency = device_desc.highspeed;
        break;
    case AP_HAL::Device::SPEED_LOW:
        frequency = device_desc.lowspeed;
        break;
    }
    derive_freq_flag(frequency);
    return true;
}

/*
  low level transfer function
 */
void SPIDevice::do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len)
{

    spiAcquireBus(spi_devices[device_desc.bus]);              /* Acquire ownership of the bus.    */

    SPIConfig spicfg = {
        NULL,
        device_desc.port,
        device_desc.pin,
        (uint16_t)(freq_flag | device_desc.mode),
        0
    };
    spiStart(spi_devices[device_desc.bus], &spicfg);       /* Setup transfer parameters.       */
    spiSelect(spi_devices[device_desc.bus]);                  /* Slave Select assertion.          */
    spiExchange(spi_devices[device_desc.bus], len,
                send, recv);          /* Atomic transfer operations.      */
    if (!cs_forced) {
        spiUnselect(spi_devices[device_desc.bus]);                /* Slave Select de-assertion.       */
    }
    spiReleaseBus(spi_devices[device_desc.bus]);              /* Ownership release.               */

}

void SPIDevice::derive_freq_flag(uint32_t _frequency)
{
    uint8_t i;
    uint32_t spi_clock_freq;
    switch(device_desc.bus) {
        case 0:
            spi_clock_freq = SPI1_CLOCK;
            break;
        case 1:
            spi_clock_freq = SPI2_CLOCK;
            break;
        case 2:
            spi_clock_freq = SPI3_CLOCK;
            break;
        case 3:
            spi_clock_freq = SPI4_CLOCK;
            break;
        default:
            spi_clock_freq = SPI1_CLOCK;
            break;
    }

    for(i = 0; i <= 7; i++) {
        spi_clock_freq /= 2;
        if (spi_clock_freq <= _frequency) {
            break;
        }
    }
    switch(i) {
        case 0: //PCLK DIV 2
            freq_flag = 0;
            break;
        case 1: //PCLK DIV 4
            freq_flag = SPI_CR1_BR_0;
            break;
        case 2: //PCLK DIV 8
            freq_flag = SPI_CR1_BR_1;
            break;
        case 3: //PCLK DIV 16
            freq_flag = SPI_CR1_BR_1 | SPI_CR1_BR_0;
            break;
        case 4: //PCLK DIV 32
            freq_flag = SPI_CR1_BR_2;
            break;
        case 5: //PCLK DIV 64
            freq_flag = SPI_CR1_BR_2 | SPI_CR1_BR_0;
            break;
        case 6: //PCLK DIV 128
            freq_flag = SPI_CR1_BR_2 | SPI_CR1_BR_1;
            break;
        case 7: //PCLK DIV 256
            freq_flag = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
            break;
    }
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (send_len == recv_len && send == recv) {
        // simplest cases, needed for DMA
        do_transfer(send, recv, recv_len);
        return true;
    }
    uint8_t buf[send_len+recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    do_transfer(buf, buf, send_len+recv_len);
    if (recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return true;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    uint8_t buf[len];
    memcpy(buf, send, len);
    do_transfer(buf, buf, len);
    memcpy(recv, buf, len);
    return true;
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &bus.semaphore;
}


AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

/*
  allow for control of SPI chip select pin
 */
bool SPIDevice::set_chip_select(bool set)
{
    cs_forced = set;
    spiSelect(spi_devices[device_desc.bus]); /* Slave Select assertion.*/
    return true;
}


/*
  return a SPIDevice given a string device name
 */
AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    /* Find the bus description in the table */
    uint8_t i;
    for (i = 0; device_table[i].name; i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (device_table[i].name == nullptr) {
        //printf("SPI: Invalid device name: %s\n", name);
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(nullptr);
    }

    SPIDesc &desc = device_table[i];

    // find the bus
    SPIBus *busp;
    for (busp = buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }
    if (busp == nullptr) {
        // create a new one
        busp = new SPIBus;
        if (busp == nullptr) {
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;

        buses = busp;
    }

    return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(*busp, desc));
}

}
