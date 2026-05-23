#define sender 1
#define testing 0


#include <stdio.h>
#include "pico/stdlib.h"
#include "RF24.h"
// #include <tusb.h>
#include "hardware/uart.h"
#include <inttypes.h>


#define PIN_NSS 1
#define PIN_IRQ 15
#define PIN_CE 14
#define PIN_SCK 2
#define PIN_MISO 0
#define PIN_MOSI 3

#define LC29HBS_UART uart1
#define LC29HBS_TX_PIN 4
#define LC29HBS_RX_PIN 5
#define LC29HBS_BAUD 115200


RF24 radio(PIN_CE, PIN_NSS);

#if sender
const uint8_t addressReceiver[6] = "BASE1";
const uint8_t addressSender[6] = "ROVER";
const bool base = true;
#else 
const uint8_t addressReceiver[6] = "ROVER";
const uint8_t addressSender[6] = "BASE1";
const bool base = false;
#endif


int initRadio(){
    spi_init(spi0, 8e6);

    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    if (!radio.begin()) {
        printf("Radio Setup Failed");
        return -1;
    }

    radio.setPALevel(RF24_PA_MAX);
    radio.enableDynamicPayloads();
    radio.setAutoAck(true);
    radio.setRetries(5, 2);
    radio.setChannel(76);
    radio.stopListening(addressSender);
    radio.openReadingPipe(1, addressReceiver);

    if (base) {
        radio.stopListening();
    } else {
        radio.startListening();
    }

    printf("Done initializing radio");

    return 0;
}

#if !testing



uint64_t getBits(const uint8_t* b, int start, int len) {
    uint64_t v = 0;
    for (int i = 0; i < len; i++) {
        int bit = start + i;
        v <<= 1;
        v |= (b[bit / 8] >> (7 - (bit % 8))) & 1;
    }
    return v;
}

int64_t getSignedBits(const uint8_t* b, int start, int len) {
    uint64_t raw = getBits(b, start, len);
    if (raw & (1ULL << (len - 1))) {
        raw |= ~((1ULL << len) - 1);
    }
    return (int64_t)raw;
}

int countBits64(uint64_t x) {
    int c = 0;
    while (x) {
        c += x & 1;
        x >>= 1;
    }
    return c;
}

int countBits32(uint32_t x) {
    int c = 0;
    while (x) {
        c += x & 1;
        x >>= 1;
    }
    return c;
}

void decodeMsmHeader(const uint8_t* p) {
    int msgType = getBits(p, 0, 12);

    bool isMsm =
        (msgType >= 1071 && msgType <= 1077) ||
        (msgType >= 1081 && msgType <= 1087) ||
        (msgType >= 1091 && msgType <= 1097) ||
        (msgType >= 1111 && msgType <= 1117) ||
        (msgType >= 1121 && msgType <= 1127);

    if (!isMsm) return;

    uint64_t satMask = getBits(p, 73, 64);
    uint32_t sigMask = getBits(p, 137, 32);

    int sats = countBits64(satMask);
    int sigs = countBits32(sigMask);

    int cells = 0;
    int cellMaskStart = 169;

    for (int i = 0; i < sats * sigs; i++) {
        cells += getBits(p, cellMaskStart + i, 1);
    }

    printf("MSM msg=%d sats=%d sigs=%d cells=%d\n",
           msgType, sats, sigs, cells);
}

void decode1005(const uint8_t* p) {
    // p should point to payload start: &rtcmBuf[3]
    int msg = getBits(p, 0, 12);
    int stationID = getBits(p, 12, 12);

    double x = getSignedBits(p, 34, 38) * 0.0001;
    double y = getSignedBits(p, 74, 38) * 0.0001;
    double z = getSignedBits(p, 114, 38) * 0.0001;

    printf("1005 station=%d ECEF X=%.4f Y=%.4f Z=%.4f meters\n",
           stationID, x, y, z);
}

uint32_t crc24q(const uint8_t *buf, int len) {
    uint32_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= ((uint32_t)buf[i]) << 16;
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000) crc ^= 0x1864CFB;
        }
    }
    return crc & 0xFFFFFF;
}

#define RTCM_MAX_LEN 1200

uint8_t rtcmBuf[RTCM_MAX_LEN];
int rtcmIndex = 0;
int rtcmExpectedLen = 0;

uint32_t rtcmCount = 0;
uint32_t badCrcCount = 0;

void parseRtcmByte(uint8_t b) {
    if (rtcmIndex == 0) {
        if (b != 0xD3) return;
        rtcmBuf[rtcmIndex++] = b;
        return;
    }

    rtcmBuf[rtcmIndex++] = b;

    if (rtcmIndex == 0) {
        if (b != 0xD3) return;
        printf("\nFOUND D3\n");
        rtcmBuf[rtcmIndex++] = b;
        return;
    }

    if (rtcmIndex == 3) {
        int len = ((rtcmBuf[1] & 0x03) << 8) | rtcmBuf[2];
        rtcmExpectedLen = len + 6;

        printf("RTCM candidate len=%d expected=%d header=%02X %02X %02X\n",
            len, rtcmExpectedLen, rtcmBuf[0], rtcmBuf[1], rtcmBuf[2]);

        if ((rtcmBuf[1] & 0xFC) != 0) {
            printf("Bad RTCM header\n");
            rtcmIndex = 0;
            rtcmExpectedLen = 0;
            return;
        }
    }

    if (rtcmExpectedLen > 0 && rtcmIndex >= rtcmExpectedLen) {
        uint32_t calc = crc24q(rtcmBuf, rtcmExpectedLen - 3);

        uint32_t got =
            ((uint32_t)rtcmBuf[rtcmExpectedLen - 3] << 16) |
            ((uint32_t)rtcmBuf[rtcmExpectedLen - 2] << 8) |
            ((uint32_t)rtcmBuf[rtcmExpectedLen - 1]);

        if (calc == got) {
            int msgType = (rtcmBuf[3] << 4) | (rtcmBuf[4] >> 4);
            int payloadLen = rtcmExpectedLen - 6;


            if (msgType == 1005) {
                decode1005(&rtcmBuf[3]);
            } else {
                decodeMsmHeader(&rtcmBuf[3]);
            }
            rtcmCount++;

            printf("RTCM msg=%d len=%d total=%lu\n",
                   msgType, payloadLen, rtcmCount);
        } else {
            badCrcCount++;
            printf("Bad RTCM CRC bad=%lu\n", badCrcCount);
        }

        rtcmIndex = 0;
        rtcmExpectedLen = 0;
    }
}

void sendCommandLC29HBS(const char* cmd) {
    uart_puts(LC29HBS_UART, cmd);
    uart_puts(LC29HBS_UART, "\r\n");
    sleep_ms(200);
}

void initLC29HBS() {
    uart_init(LC29HBS_UART, LC29HBS_BAUD);
    gpio_set_function(LC29HBS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LC29HBS_RX_PIN, GPIO_FUNC_UART);
    // sendCommandLC29HBS("$PAIR432,1*22");
    // sendCommandLC29HBS("$PAIR434,1*24");
    // sendCommandLC29HBS("$PQTMSAVEPAR*5A");
}

uint8_t packet[32];
int packetIndex = 0;
uint32_t totalBytes = 0;
absolute_time_t lastPrintTime = get_absolute_time();
absolute_time_t lastByteTime = get_absolute_time();

void calcStats() {
    if(base) {
        if (absolute_time_diff_us(lastPrintTime, get_absolute_time()) >= 1e6) {
            printf("Sent bytes/s: %d\n", totalBytes);
            totalBytes = 0;
            lastPrintTime = get_absolute_time();
        }
    }
    else {
        if (absolute_time_diff_us(lastPrintTime, get_absolute_time()) >= 1e6) {
            printf("Received bytes/s: %d\n", totalBytes);
            totalBytes = 0;
            lastPrintTime = get_absolute_time();
        }
    }
}

uint8_t rtcmMessage[8192];
int rtcmMessageIndex = 0;

int mainLoop() {
    if (base) {
        while (uart_is_readable(LC29HBS_UART)) {
            uint8_t b = uart_getc(LC29HBS_UART);
            parseRtcmByte(b);
            rtcmMessage[rtcmMessageIndex] = b;
            rtcmMessageIndex++;
            lastByteTime = get_absolute_time();
            if (rtcmMessageIndex >= 8192) {
                printf("rtcm frame overrun");
                break;
            }
            totalBytes ++;
            calcStats();
        }
        if (rtcmMessageIndex > 0 && absolute_time_diff_us(lastByteTime, get_absolute_time()) > 10e3) {
            for (int i = 0; i + 32 <= rtcmMessageIndex; i += 32) {
                radio.write(&rtcmMessage[i], 32);
            }
            if (rtcmMessageIndex % 32 != 0) {
                radio.write(&rtcmMessage[rtcmMessageIndex - rtcmMessageIndex%32], rtcmMessageIndex%32);
            }
            rtcmMessageIndex = 0;
        }
    } else {
        uint8_t pipe;
        while (radio.available(&pipe)) {
            uint8_t bytes = radio.getDynamicPayloadSize();
            radio.read(&packet, bytes);
            totalBytes += bytes;
            calcStats();
        }
    }
    return 0;
}

int main() {
    stdio_init_all();

    initRadio();

    if (base) {
        initLC29HBS();
    }

    while (true) {
        int result = mainLoop();
        if (result == -1) {
            return -1;
        }
        calcStats();
    }
}

#else

uint8_t payload[32];
uint64_t totalBytes = 0;
absolute_time_t lastPrintTime = 0;


int mainLoop() {
    if (base) {
        absolute_time_t start = get_absolute_time();
        bool report = radio.write(&payload, sizeof(payload));
        absolute_time_t stop = get_absolute_time();

        if (report) {
            totalBytes += 32;
        } else {
            printf("Fail %" PRId64 "\n", absolute_time_diff_us(start, stop));
        }
        if (absolute_time_diff_us(lastPrintTime, get_absolute_time()) >= 1e6) {
            printf("Throughput Bytes/s: %" PRId64 "\n", totalBytes);
            totalBytes = 0;
            lastPrintTime = get_absolute_time();
        }
    } else {
        uint8_t pipe;
        if (radio.available(&pipe)) {
            uint8_t bytes = radio.getPayloadSize();
            radio.read(&payload, 32);
            totalBytes += 32;
        }
        if (absolute_time_diff_us(lastPrintTime, get_absolute_time()) >= 1e6) {
            printf("Throughput Bytes/s: %" PRId64 "\n", totalBytes);
            totalBytes = 0;
            lastPrintTime = get_absolute_time();
        }
    }
    return 0;
}



int main() {
    stdio_init_all();

    if (initRadio() == -1) {
        return -1;
    }

    if (testing) {
        for (int i = 0; i < 8; i ++) {
            payload[i] = i;
        }
    }

    while (true) {
        int mainLoopResult = mainLoop();
        if (mainLoopResult == -1) {
            return -1;
        }
        sleep_ms(1);
    }
    
}

#endif