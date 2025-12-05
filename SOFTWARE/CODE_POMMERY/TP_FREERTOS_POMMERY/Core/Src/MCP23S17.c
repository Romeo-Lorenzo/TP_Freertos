/*
 * MCP23S17.c
 *
 *  Created on: Nov 13, 2025
 *      Author: mathi
 */


#include "MCP23S17.h"





// Initialize MCP23S17

void MCP23S17_SetAllPinsLow(void) {
MCP23S17_WriteRegister(MCP23S17_GPIOA, 0x0); // Set all GPIOA high
MCP23S17_WriteRegister(MCP23S17_GPIOB, 0x0); // Set all GPIOB high

}

// Write to a register
void MCP23S17_WriteRegister(uint8_t reg, uint8_t value) {
uint8_t txData[3] = {MCP23S17_WRITE, reg, value};

HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_RESET);
HAL_SPI_Transmit(&hspi3, txData, 3, HAL_MAX_DELAY);
HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_SET);
}

// Read from a register (optional, for verification)
uint8_t MCP23S17_ReadRegister(uint8_t reg) {
uint8_t txData[3] = {MCP23S17_READ, reg, 0x00};
uint8_t rxData[3];

HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_RESET);
HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 3, HAL_MAX_DELAY);
HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_SET);

return rxData[2];
}

// Set all pins to high



void MCP23S17_SetPin(uint8_t pin, uint8_t state)
{
    if (pin > 15) return;

    uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t bit = (pin < 8) ? pin : (pin - 8);

    // Lire l'état actuel du port
    uint8_t current = MCP23S17_ReadRegister(reg);

    if (state) {
        // mettre le bit à 1
        current |= (1u << bit);
    } else {
        // mettre le bit à 0
        current &= ~(1u << bit);
    }

    MCP23S17_WriteRegister(reg, current);
}

void MCP23S17_Init(void) {
// Set CS high initially
HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_SET);

// Obligatoire
HAL_GPIO_WritePin(MCP_RESET_GPIO_PORT, MCP_RESET_PIN, GPIO_PIN_RESET);
HAL_Delay(1);
HAL_GPIO_WritePin(MCP_RESET_GPIO_PORT, MCP_RESET_PIN, GPIO_PIN_SET);
HAL_Delay(1);

// Configure IOCON: Sequential operation disabled, etc. (default is fine, but set explicitly)
MCP23S17_WriteRegister(MCP23S17_IOCONA, 0x20); // BANK=0, SEQOP=0, etc.

// Set all pins as outputs
MCP23S17_WriteRegister(MCP23S17_IODIRA, 0x00); // All GPIOA outputs
MCP23S17_WriteRegister(MCP23S17_IODIRB, 0x00); // All GPIOB outputs

for(int i=0; i<16;i++){
	MCP23S17_SetPin(i, 0);

}
}



