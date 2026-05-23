// #define sender 0


// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "LoRa-RP2040.h"
// #include <string.h>


// #define PIN_NSS 1
// #define PIN_RST 15
// #define PIN_DIO0 14

// #define SPREADING_FACTOR 7
// #define SIGNAL_BANDWIDTH 500E3
// #define FREQUENCY 433E6
// #define CODING_RATE 5
// #define SYNC_WORD 0x12
// #define TX_POWER 17

// void initLoRa() {
//     LoRa.setPins(PIN_NSS, PIN_RST, PIN_DIO0);

//     if (!LoRa.begin(FREQUENCY)) {
//         printf("Failed LoRa initialization");
//         return;
//     }

//     LoRa.setSpreadingFactor(SPREADING_FACTOR);
//     LoRa.setSignalBandwidth(SIGNAL_BANDWIDTH);
//     LoRa.setFrequency(FREQUENCY);
//     LoRa.setCodingRate4(CODING_RATE);
//     // LoRa.setSyncWord(SYNC_WORD);
//     LoRa.setTxPower(TX_POWER);
// }

// #if sender

// int main() {
//     stdio_init_all();

//     initLoRa();

//     uint8_t buffer[255];

//     for (int i = 0; i < 255; i++) {
//         buffer[i] = i;   
//     }

//     while (true) {
//         LoRa.beginPacket();
//         LoRa.write(buffer, 255);
//         LoRa.endPacket();
//         printf("sent\n");
//         sleep_ms(1);
//     }
    
// }

// #elif !sender


// int main() {
//     stdio_init_all();

//     initLoRa();
//     LoRa.receive();

//     uint8_t buffer[256];
//     absolute_time_t lastTime = get_absolute_time();
//     int totalBytes = 0;
//     while (true) {
//         // LoRa.receive();
//         int packetSize = LoRa.parsePacket();

//         if (packetSize) {
//             int i = 0;
//             while (LoRa.available() && i < 256) {
//                 buffer[i] = LoRa.read();
//                 i++;
//             }
//             totalBytes += i;
//         }

//         if (absolute_time_diff_us(lastTime, get_absolute_time()) >= 1'000'000) {
//             printf("Bytes/sec: %d\n", totalBytes);
//             totalBytes = 0;
//             lastTime = get_absolute_time();
//         }
//     }
// }

// #endif

int main() {
    
}