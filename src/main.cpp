#include <Arduino.h>

#include "adc_setup.h"
#include "blocks_lib.h"
#include "dma_setup.h"
#include "encoder.h"
#include "gpio_setup.h"
#include "interrupt_setup.h"
#include "spi_setup.h"
#include "system_clock.h"
#include "timerPWM.h"

#include "blocks_lib.h"
#include "foc_setup.h"

#include <SPI.h>

// COM PORT MUST BE OPENED TO RUN PROGRAM !!!!!!!!!!!!!!!!!

// SPIClass SPI_1(PA7, PA6, PA5);
// SPISettings ENC_SPI_SETTINGS(8000000, MSBFIRST, SPI_MODE1);

// void initEncoderCallback() {
//     SPI_1.begin();
//     SPI_1.beginTransaction(ENC_SPI_SETTINGS);
//     pinMode(PC4, OUTPUT);
//     digitalWrite(PC4, HIGH);
// }

// uint32_t readEncoderCallback() {
//     digitalWriteFast(PC_4, LOW);         // start spi
//     SPI_1.transfer16(0x8020);            // Send command word
//     respond = SPI_1.transfer16(0x0000);  // Recieve position
//     digitalWriteFast(PC_4, HIGH);        // end spi
//     return respond << 17;                // normalize
// }

void setup() {
    IO_Init();
    NVIC_Init();
    DMA_Init();
    ADC_Init();
    PWM_Init();
    SPI1N::Init();
    ENCODER::Init();

    digitalWriteFast(PB_2, HIGH);
    digitalWriteFast(PA_4, HIGH);

    // initEncoderCallback();

    SerialUSB.begin();    // Initialize SerialUSB
    while (!SerialUSB) {  // Wait for SerialUSB connection
        ;
    }
    SerialUSB.println("Ready!");

    MOTOR_CONTROL::resistance = 3500;               // Set motor phase resistance in mOhms
    MOTOR_CONTROL::current_target_polar.rad = 500;  // Set motor phase current in mA
}

void loop() {
    ENCODER::RequestUpdate();
    uint16_t angle = ENCODER::angle;
    // MOTOR_CONTROL::angleRaw = readEncoderCallback();
}
