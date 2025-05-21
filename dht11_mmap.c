#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

// GPIO2 Base アドレス（GPIO2_Ax）
#define RK3328_GPIO2_BASE 0xFF230000
#define GPIO_MEM_SIZE     0x100
#define GPIO_PIN          2  // GPIO2_A2 = GPIO66
#define DATA_BITS         40

volatile uint32_t *gpio_base = NULL;

// GPIO レジスタ
#define GPIO_SWPORTA_DR   0x00
#define GPIO_SWPORTA_DDR  0x04
#define GPIO_EXT_PORTA    0x50

void delay_us(unsigned int us) {
    struct timespec ts_start, ts_now;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts_start);
    do {
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts_now);
    } while (((ts_now.tv_sec - ts_start.tv_sec) * 1000000 +
             (ts_now.tv_nsec - ts_start.tv_nsec) / 1000) < us);
}

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

int read_dht11_data(int *humidity, int *temperature) {
    uint8_t data[5] = {0};

    // スタートシーケンス
    set_gpio_output();
    gpio_write(0);
    delay_us(20000);  // 20ms
    gpio_write(1);
    delay_us(40);     // 40us
    set_gpio_input();

    // 応答待ち: LOW → HIGH
    int timeout = 0;
    while (gpio_read()) {
        delay_us(1);
        if (++timeout > 100) {
            printf("Timeout waiting for LOW (start)\n");
            return -1;
        }
    }

    timeout = 0;
    while (!gpio_read()) {
        delay_us(1);
        if (++timeout > 100) {
            printf("Timeout waiting for HIGH (response)\n");
            return -1;
        }
    }

    timeout = 0;
    while (gpio_read()) {
        delay_us(1);
        if (++timeout > 100) {
            printf("Timeout waiting for LOW (data start)\n");
            return -1;
        }
    }

    // ビット読み取り（40ビット = 5バイト）
    for (int i = 0; i < 40; i++) {
        // LOW: 開始ビット (約50us)
        timeout = 0;
        while (!gpio_read()) {
            delay_us(1);
            if (++timeout > 100) {
                printf("Timeout waiting for HIGH at bit %d\n", i);
                return -1;
            }
        }

        // HIGHパルス長で0/1を判別
        delay_us(35);  // 約26〜28us: 0, 70us: 1

        if (gpio_read()) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }

        // HIGHが続く場合、LOWに戻るまで待つ
        timeout = 0;
        while (gpio_read()) {
            delay_us(1);
            if (++timeout > 100) break;
        }
    }

    // チェックサム確認
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (data[4] != sum) {
        printf("Checksum error: got %d, expected %d\n", data[4], sum);
        return -1;
    }

    *humidity = data[0];
    *temperature = data[2];
    return 0;
}

int main() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    gpio_base = (uint32_t *)mmap(NULL, GPIO_MEM_SIZE, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, RK3328_GPIO2_BASE);
    if (gpio_base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    int humidity = 0, temperature = 0;
    int retries = 5;
    while (retries-- > 0) {
        if (read_dht11_data(&humidity, &temperature) == 0) {
            printf("成功: 湿度 %d%%, 温度 %d°C\n", humidity, temperature);
            break;
        }
        printf("再試行中...\n");
        sleep(1); // DHT11の仕様では1秒以上空ける必要あり
    }

    munmap((void *)gpio_base, GPIO_MEM_SIZE);
    close(fd);
    return 0;
}
