//
// gcc -O2 -o dht22_mmap dht22_mmap.c
// sudo ./dht22_mmap
//
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

//#define _DEBUG

// GPIO2 Base アドレス（NanoPi NEO3: GPIO2_Ax）
#define RK3328_GPIO2_BASE 0xFF230000
#define GPIO_MEM_SIZE     0x100
#define GPIO_PIN          2  // GPIO2_A2 = GPIO66
#define DATA_BITS         40

volatile uint32_t *gpio_base = NULL;

// GPIO レジスタオフセット
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
    gpio_base[GPIO_SWPORTA_DDR / 4] |= (1 << GPIO_PIN);
}

void set_gpio_input() {
    gpio_base[GPIO_SWPORTA_DDR / 4] &= ~(1 << GPIO_PIN);
}

void gpio_write(int val) {
    if (val)
        gpio_base[GPIO_SWPORTA_DR / 4] |= (1 << GPIO_PIN);
    else
        gpio_base[GPIO_SWPORTA_DR / 4] &= ~(1 << GPIO_PIN);
}

int gpio_read() {
    return (gpio_base[GPIO_EXT_PORTA / 4] & (1 << GPIO_PIN)) != 0;
}

int read_dht22_data(float *humidity, float *temperature) {
    uint8_t data[5] = {0};

    // スタート信号
    set_gpio_output();
    gpio_write(0);
    delay_us(10000); // 10ms
    gpio_write(1);
    delay_us(40);    // 40us
    set_gpio_input();

    // 応答 LOW
    int timeout = 0;
    while (gpio_read()) {
        delay_us(1);
       if (++timeout > 200) {
#ifdef _DEBUG        
        printf("Timeout waiting for LOW (start)\n");
#endif        
        return -1;
       }
    }

    // 応答 HIGH
    timeout = 0;
    while (!gpio_read()) {
        delay_us(1);
        if (++timeout > 200) {
#ifdef _DEBUG        
            printf("Timeout waiting for HIGH (response)\n");
#endif
            return -1;
        }
    }

    // データ開始 LOW
    timeout = 0;
    while (gpio_read()) {
        delay_us(1);
        if (++timeout > 200) {
#ifdef _DEBUG        
            printf("Timeout waiting for LOW (data start)\n");
#endif
            return -1;
        }
    }

    // 40ビット受信
    for (int i = 0; i < 40; i++) {
        timeout = 0;
        while (!gpio_read()) {
            delay_us(1);
            if (++timeout > 200) {
#ifdef _DEBUG        
                printf("Timeout waiting for HIGH at bit %d\n", i);
#endif
                return -1;
            }
        }

        delay_us(45); // パルス幅を判定（"0" or "1"）

        if (gpio_read()) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }

        timeout = 0;
        while (gpio_read()) {
            delay_us(1);
            if (++timeout > 200) break;
        }
    }

    // チェックサム確認
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (data[4] != sum) {
#ifdef _DEBUG        
        printf("Checksum error: got %d, expected %d\n", data[4], sum);
#endif
        return -1;
    }

    *humidity = ((data[0] << 8) | data[1]) / 10.0;
    *temperature = (((data[2] & 0x7F) << 8) | data[3]) / 10.0;
    if (data[2] & 0x80) *temperature *= -1;

    return 0;
}

void print_result(float humidity, float temperature) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char buf[256];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", t);
    printf("%s : Humidity: %.1f%%, Temp: %.1f°C\n", buf, humidity, temperature);
}

int check_sensor_presence() {
    // スタートシーケンスの直前にLOWに引く
    set_gpio_output();
    gpio_write(0);
    delay_us(1000);  // 1ms
    gpio_write(1);
    delay_us(40);
    set_gpio_input();

    // DHTがLOWで応答するか確認
    int timeout = 0;
    while (gpio_read()) {
        delay_us(1);
        if (++timeout > 300) {
            return 0;  // HIGHのまま → 応答なし（未接続の可能性）
        }
    }
    return 1;  // 応答あり
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

        // センサーの接続チェック
    if (!check_sensor_presence()) {
        fprintf(stderr, "DHTセンサーが接続されていないか、応答がありません。\n");
        munmap((void *)gpio_base, GPIO_MEM_SIZE);
        close(fd);
        return 1;
    }
    
    float humidity = 0, temperature = 0;
    int retries = 5;
    while (retries-- > 0) {
        if (read_dht22_data(&humidity, &temperature) == 0) {
            print_result(humidity, temperature);
            break;
        }
        sleep(1); // DHT22: 最低 1秒インターバル
    }

    munmap((void *)gpio_base, GPIO_MEM_SIZE);
    close(fd);
    return 0;
}
