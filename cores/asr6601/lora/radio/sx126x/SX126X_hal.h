#pragma once

/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian

Heavily modified/simplified by Alessandro Carcione 2020 for ELRS project 
*/

#include "SX126X_Regs.h"

enum SX126X_BusyState_
{
    SX1280_NOT_BUSY = true,
    SX1280_BUSY = false,
};

class SX126XHal
{
public:
    static SX126XHal *instance;

    SX126XHal();

    void init();
    void end();
    void reset();

    void WriteCommand(SX126X_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void WriteCommand(SX126X_RadioCommands_t command, uint8_t val);
    void WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    void WriteRegister(uint16_t address, uint8_t value);

    void ReadCommand(SX126X_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    uint8_t ReadRegister(uint16_t address);

    void WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size); // Writes and Reads to FIFO
    void ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size);

    bool WaitOnBusy();
    
    void TXenable();
    void RXenable();
    void TXRXdisable();

    static void dioISR();
    void (*IsrCallback)(); //function pointer for callback
};
