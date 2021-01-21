/*
 * THIS FILE IS AUTOMATICALLY GENERATED AND MUST NOT BE EDITED MANUALLY!
 *
 * I2C-Generator: 0.2.0
 * Yaml Version: 0.3.0
 * Template Version: 0.2.0
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "SensirionI2CSvm40.h"
#include "Arduino.h"
#include "SensirionCoreArduinoLibrary.h"
#include <Wire.h>

#define SVM40_I2C_ADDRESS 106

SensirionI2CSvm40::SensirionI2CSvm40() {
}

void SensirionI2CSvm40::begin(TwoWire& i2cBus) {
    _i2cBus = &i2cBus;
}

uint16_t SensirionI2CSvm40::startContinuousMeasurement() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0x10);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(1);
    return error;
}

uint16_t SensirionI2CSvm40::stopMeasurement() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0x104);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(50);
    return error;
}

uint16_t SensirionI2CSvm40::readMeasuredValuesAsIntegers(int16_t& vocIndex,
                                                         int16_t& humidity,
                                                         int16_t& temperature) {
    uint16_t error;
    uint8_t buffer[9];
    SensirionI2CTxFrame txFrame(buffer, 9);

    error = txFrame.addCommand(0x3A6);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 9);
    error = SensirionI2CCommunication::receiveFrame(SVM40_I2C_ADDRESS, 9,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(vocIndex);
    error |= rxFrame.getInt16(humidity);
    error |= rxFrame.getInt16(temperature);
    return error;
}

uint16_t SensirionI2CSvm40::readMeasuredValuesAsIntegersWithRawParameters(
    int16_t& vocIndex, int16_t& humidity, int16_t& temperature,
    uint16_t& rawVocTicks, int16_t& rawHumidity, int16_t& rawTemperature) {
    uint16_t error;
    uint8_t buffer[18];
    SensirionI2CTxFrame txFrame(buffer, 18);

    error = txFrame.addCommand(0x3B0);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 18);
    error = SensirionI2CCommunication::receiveFrame(SVM40_I2C_ADDRESS, 18,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(vocIndex);
    error |= rxFrame.getInt16(humidity);
    error |= rxFrame.getInt16(temperature);
    error |= rxFrame.getUInt16(rawVocTicks);
    error |= rxFrame.getInt16(rawHumidity);
    error |= rxFrame.getInt16(rawTemperature);
    return error;
}

uint16_t
SensirionI2CSvm40::setTemperatureOffsetForRhtMeasurements(int16_t tOffset) {
    uint16_t error;
    uint8_t buffer[5];
    SensirionI2CTxFrame txFrame(buffer, 5);

    error = txFrame.addCommand(0x6014);
    error |= txFrame.addInt16(tOffset);
    if (error) {
        return error;
    }

    return SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                *_i2cBus);
}

uint16_t
SensirionI2CSvm40::getTemperatureOffsetForRhtMeasurements(int16_t& tOffset) {
    uint16_t error;
    uint8_t buffer[3];
    SensirionI2CTxFrame txFrame(buffer, 3);

    error = txFrame.addCommand(0x6014);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 3);
    error = SensirionI2CCommunication::receiveFrame(SVM40_I2C_ADDRESS, 3,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(tOffset);
    return error;
}

uint16_t SensirionI2CSvm40::setVocAlgorithmTuningParameters(
    int16_t vocIndexOffset, int16_t learningTimeHours,
    int16_t gatingMaxDurationMinutes, int16_t stdInitial) {
    uint16_t error;
    uint8_t buffer[14];
    SensirionI2CTxFrame txFrame(buffer, 14);

    error = txFrame.addCommand(0x6083);
    error |= txFrame.addInt16(vocIndexOffset);
    error |= txFrame.addInt16(learningTimeHours);
    error |= txFrame.addInt16(gatingMaxDurationMinutes);
    error |= txFrame.addInt16(stdInitial);
    if (error) {
        return error;
    }

    return SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                *_i2cBus);
}

uint16_t SensirionI2CSvm40::getVocAlgorithmTuningParameters(
    int16_t& vocIndexOffset, int16_t& learningTimeHours,
    int16_t& gatingMaxDurationMinutes, int16_t& stdInitial) {
    uint16_t error;
    uint8_t buffer[12];
    SensirionI2CTxFrame txFrame(buffer, 12);

    error = txFrame.addCommand(0x6083);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 12);
    error = SensirionI2CCommunication::receiveFrame(SVM40_I2C_ADDRESS, 12,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(vocIndexOffset);
    error |= rxFrame.getInt16(learningTimeHours);
    error |= rxFrame.getInt16(gatingMaxDurationMinutes);
    error |= rxFrame.getInt16(stdInitial);
    return error;
}

uint16_t SensirionI2CSvm40::storeNvData() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0x6002);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(500);
    return error;
}

uint16_t SensirionI2CSvm40::setVocAlgorithmState(uint8_t state[],
                                                 uint8_t stateSize) {
    uint16_t error;
    uint8_t buffer[14];
    SensirionI2CTxFrame txFrame(buffer, 14);

    error = txFrame.addCommand(0x6181);
    error |= txFrame.addBytes(state, stateSize);
    if (error) {
        return error;
    }

    return SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                *_i2cBus);
}

uint16_t SensirionI2CSvm40::getVocAlgorithmState(uint8_t state[],
                                                 uint8_t stateSize) {
    uint16_t error;
    uint8_t buffer[12];
    SensirionI2CTxFrame txFrame(buffer, 12);

    error = txFrame.addCommand(0x6181);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 12);
    error = SensirionI2CCommunication::receiveFrame(SVM40_I2C_ADDRESS, 12,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getBytes(state, stateSize);
    return error;
}

uint16_t
SensirionI2CSvm40::getVersion(uint8_t& firmwareMajor, uint8_t& firmwareMinor,
                              bool& firmwareDebug, uint8_t& hardwareMajor,
                              uint8_t& hardwareMinor, uint8_t& protocolMajor,
                              uint8_t& protocolMinor) {
    uint16_t error;
    uint8_t buffer[12];
    SensirionI2CTxFrame txFrame(buffer, 12);

    error = txFrame.addCommand(0xD100);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 12);
    error = SensirionI2CCommunication::receiveFrame(SVM40_I2C_ADDRESS, 12,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt8(firmwareMajor);
    error |= rxFrame.getUInt8(firmwareMinor);
    error |= rxFrame.getBool(firmwareDebug);
    error |= rxFrame.getUInt8(hardwareMajor);
    error |= rxFrame.getUInt8(hardwareMinor);
    error |= rxFrame.getUInt8(protocolMajor);
    error |= rxFrame.getUInt8(protocolMinor);
    uint8_t padding;
    error |= rxFrame.getUInt8(padding);  // remove padding
    return error;
}

uint16_t SensirionI2CSvm40::getSerialNumber(uint8_t serialNumber[],
                                            uint8_t serialNumberSize) {
    uint16_t error;
    uint8_t buffer[39];
    SensirionI2CTxFrame txFrame(buffer, 39);

    error = txFrame.addCommand(0xD033);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 39);
    error = SensirionI2CCommunication::receiveFrame(SVM40_I2C_ADDRESS, 39,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getBytes(serialNumber, serialNumberSize);
    return error;
}

uint16_t SensirionI2CSvm40::deviceReset() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0xD304);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SVM40_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(100);
    return error;
}
