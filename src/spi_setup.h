#ifndef SPI_SETUP_H
#define SPI_SETUP_H

#include <Arduino.h>  // Include the Arduino library
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_spi.h"

typedef void (*SPI_RX_Callback)(uint16_t);
typedef void (*SPI_TX_Callback)();

namespace SPI1N {

SPI_RX_Callback RxCallback = nullptr;
SPI_TX_Callback TxCallback = nullptr;

void SetCallbacks(SPI_RX_Callback RxCallback_, SPI_TX_Callback TxCallback_) {
    RxCallback = RxCallback_;
    TxCallback = TxCallback_;
}

void Init() {
    LL_SPI_InitTypeDef SPI_InitStruct = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    /* SPI1 parameter configuration*/
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 7;
    LL_SPI_Init(SPI1, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_DisableNSSPulseMgt(SPI1);

    // Enable SPI Interrupts
    // LL_SPI_EnableIT_RXNE(SPI1);
    LL_SPI_EnableIT_TXE(SPI1);
    // LL_SPI_EnableIT_ERR(SPI1);
}

void ChipSelect(bool select) {
    select ? LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4) : LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);
}

void Start() {
    ChipSelect(false);
    LL_SPI_Enable(SPI1);
}

void TransmitData16(uint16_t data) {
    LL_SPI_TransmitData16(SPI1, data);
}

}  // namespace SPI1N

extern "C" void SPI1_IRQHandler(void) {
    /* Check RXNE flag value in ISR register */
    if (LL_SPI_IsActiveFlag_RXNE(SPI1)) {
        if (SPI1N::RxCallback) {
            SPI1N::RxCallback(LL_SPI_ReceiveData16(SPI1));
        }
    }
    /* Check RXNE flag value in ISR register */
    else if (LL_SPI_IsActiveFlag_TXE(SPI1)) {
        if (SPI1N::TxCallback) {
            SPI1N::TxCallback();
        }
    }
    /* Check STOP flag value in ISR register */
    // else if (LL_SPI_IsActiveFlag_OVR(SPI1)) {
    //     // Call Error function
    // }
}

#endif  // SPI_SETUP_H
