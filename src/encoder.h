#ifndef ENCODER_H
#define ENCODER_H

#include "spi_setup.h"

namespace ENCODER {

enum class STATE { IDLE, SENDING_ADDRESS, RECEIVING_DATA };

volatile STATE state = STATE::IDLE;
volatile uint16_t angle = 0;

// void FSM(uint16_t data) {
//     switch (state) {
//         case STATE::IDLE:
//             state = STATE::SENDING_ADDRESS;
//             SPI1N::ChipSelect(true);
//             SPI1N::TransmitData16(0x8020);
//             break;

//         case STATE::SENDING_ADDRESS:
//             state = STATE::RECEIVING_DATA;
//             SPI1N::TransmitData16(0x0000);
//             break;

//         case STATE::RECEIVING_DATA:
//             state = STATE::IDLE;
//             angle = data;
//             SPI1N::ChipSelect(false);
//             break;
//     }
// }

// void DataReceivedHandler(uint16_t data) {
//     FSM(data);
// }

// void Init() {
//     SPI1N::SetCallbacks(DataReceivedHandler, nullptr);
//     SPI1N::Start();
// }

// void RequestUpdate() {
//     if (state != STATE::IDLE) {
//         return;
//     }

//     FSM(0);
// }

}  // namespace ENCODER

#endif  // ENCODER_H
