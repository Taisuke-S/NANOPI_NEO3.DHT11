#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

#define RK3328_GPIO2_BASE 0xFF230000  

//#define GPIO_MEM_SIZE     0x10000     // 64KB
#define GPIO_MEM_SIZE 0x100
#define GPIO_PIN          2           // GPIO2_A2 = GPIO66
#define DATA_BITS         40
#define DELAY         30
#define TiMEOUT         2500

#define GET_BIT(value, bit) (((value) >> (bit)) & 1)

volatile uint32_t *gpio_base = NULL;

void delay_us(unsigned int us) {
    struct timespec ts_start, ts_now;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts_start);
    do {
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts_now);
    } while (((ts_now.tv_sec - ts_start.tv_sec) * 1000000 +
             (ts_now.tv_nsec - ts_start.tv_nsec) / 1000) < us);
}

// GPIO register macros (RK3328)
#define GPIO_SWPORTA_DR      0x00
#define GPIO_SWPORTA_DDR     0x04
#define GPIO_EXT_PORTA       0x50

void set_gpio_output() {
    uint32_t val = gpio_base[GPIO_SWPORTA_DDR / 4];
    val |= (1 << GPIO_PIN);
    gpio_base[GPIO_SWPORTA_DDR / 4] = val;
}

void set_gpio_input() {
    uint32_t val = gpio_base[GPIO_SWPORTA_DDR / 4];
    val &= ~(1 << GPIO_PIN);
    gpio_base[GPIO_SWPORTA_DDR / 4] = val;
}

void gpio_write(int val) {
    uint32_t reg = gpio_base[GPIO_SWPORTA_DR / 4];
    if (val)
        reg |= (1 << GPIO_PIN);
    else
        reg &= ~(1 << GPIO_PIN);
    gpio_base[GPIO_SWPORTA_DR / 4] = reg;
}

int gpio_read() {
    return (gpio_base[GPIO_EXT_PORTA / 4] & (1 << GPIO_PIN)) != 0;
}


int main() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    gpio_base = mmap(NULL, GPIO_MEM_SIZE, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, RK3328_GPIO2_BASE);

    set_gpio_output();

    while (1) {
        gpio_write(0);  // LOW
        printf("write 0\n");
        usleep(500000);
        gpio_write(1);  // HIGH
        printf("write 1\n");
        usleep(500000);
    }
}
