
#include "flash.h"
#include "hal.h"

//NOTE: this driver is modified to work with stm32f412

#define KB(x)   ((x*1024))
// Refer Flash memory map in the User Manual to fill the following fields per microcontroller
#define STM32_FLASH_BASE    0x08000000
#define STM32_FLASH_NPAGES  13
#define STM32_FLASH_SIZE    KB(1024)
const uint32_t flash_memmap[] = { KB(16), KB(16), KB(16), KB(16), KB(64),
                                  KB(128), KB(128), KB(128), KB(128), KB(128), KB(128), KB(128)};

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB
/* Some compiler options will convert short loads and stores into byte loads
 * and stores.  We don't want this to happen for IO reads and writes!
 */
/* # define getreg16(a)       (*(volatile uint16_t *)(a)) */
static inline uint16_t getreg16(unsigned int addr)
{
  uint16_t retval;
 __asm__ __volatile__("\tldrh %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
  return retval;
}

/* define putreg16(v,a)       (*(volatile uint16_t *)(a) = (v)) */
static inline void putreg16(uint16_t val, unsigned int addr)
{
 __asm__ __volatile__("\tstrh %0, [%1]\n\t": : "r"(val), "r"(addr));
}

uint32_t stm32_flash_getpageaddr(uint32_t page)
{
    uint32_t base_address = STM32_FLASH_BASE;
    uint32_t i;

    if (page >= STM32_FLASH_NPAGES) {
        return 0;
    }

    for (i = 0; i < page; ++i) {
        base_address += stm32_flash_getpagesize(i);
    }

    return base_address;
}

uint32_t stm32_flash_getpagesize(uint32_t page)
{
    return flash_memmap[page];
}

uint32_t stm32_flash_getnumpages()
{
    return STM32_FLASH_NPAGES;
}

void stm32_flash_unlock(void)
{
    while (FLASH->SR & FLASH_SR_BSY) {
    }

    if (FLASH->CR & FLASH_CR_LOCK) {
        /* Unlock sequence */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

void stm32_flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}


int16_t stm32_flash_getpage(uint32_t addr)
{
    size_t page_end = 0;
    size_t i;

    if (addr >= STM32_FLASH_BASE) {
          addr -= STM32_FLASH_BASE;
    }

    if (addr >= STM32_FLASH_SIZE) {
      return -1;
    }

    for (i = 0; i < STM32_FLASH_NPAGES; ++i) {
        page_end += stm32_flash_getpagesize(i);
        if (page_end > addr) {
            return i;
        }
    }

    return -1;
}

int16_t stm32_flash_erasepage(uint32_t page)
{

    if (page >= STM32_FLASH_NPAGES) {
        return -1;
    }

    /* Get flash ready and begin erasing single page */

    if (!(RCC->CR & RCC_CR_HSION)) {
        return -1;
    }

    stm32_flash_unlock();

    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_0;

    FLASH->CR &= ~(FLASH_CR_SNB);
    FLASH->CR |= ((page) << 3);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    //Note: Do something nice here!!
    while (FLASH->SR & FLASH_SR_BSY){
    }

    FLASH->CR &= ~(FLASH_CR_SER);

    /* Verify */
    if (stm32_flash_ispageerased(page) == 0) {
        return stm32_flash_getpagesize(page); /* success */
    } else {
        return -1; /* failure */
    }
}

int16_t stm32_flash_ispageerased(uint32_t page)
{
    uint32_t addr;
    uint32_t count;
    uint32_t bwritten = 0;

    if (page >= STM32_FLASH_NPAGES) {
        return -1;
    }

    /* Verify */

    for (addr = stm32_flash_getpageaddr(page), count = stm32_flash_getpagesize(page);
        count; count--, addr++) {
        if ((*(volatile uint8_t *)(addr)) != 0xff) {
            bwritten++;
        }
    }

    return bwritten;
}

int16_t stm32_flash_write(uint32_t addr, const void *buf, uint32_t count)
{
    uint16_t *hword = (uint16_t *)buf;
    uint32_t written = count;

    /* STM32 requires half-word access */
    if (count & 1) {
        return -1;
    }

    /* Check for valid address range */

    if (addr >= STM32_FLASH_BASE) {
        addr -= STM32_FLASH_BASE;
    }

    if ((addr+count) >= STM32_FLASH_SIZE) {
        return -1;
    }

    /* Get flash ready and begin flashing */

    if (!(RCC->CR & RCC_CR_HSION)) {
        return -1;
    }

    stm32_flash_unlock();


    /* TODO: implement up_progmem_write() to support other sizes than 16-bits */
    FLASH->CR &= ~(FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_CR_PSIZE_0 | FLASH_CR_PG;

  for (addr += STM32_FLASH_BASE; count; count -= 2, hword++, addr += 2)
    {
      /* Write half-word and wait to complete */

        putreg16(*hword, addr);

        //Note: Do something nice here!!
        while (FLASH->SR & FLASH_SR_BSY) {
        }

        /* Verify */

        if (FLASH->SR & FLASH_SR_WRPERR) {
            FLASH->CR &= ~(FLASH_CR_PG);
            return -1;
        }

        if (getreg16(addr) != *hword) {
            FLASH->CR &= ~(FLASH_CR_PG);
            return -1;
        }
    }

    FLASH->CR &= ~(FLASH_CR_PG);
    return written;
}
