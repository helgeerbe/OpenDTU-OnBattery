// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include "Configuration.h"
#include "Battery.h"
#include <driver/twai.h>
#include <Arduino.h>
#include <memory>

struct BatteryCanReceiver : public BatteryProvider {
public:
    bool init(bool verboseLogging, char* providerName);
    void deinit() final;
    void loop() final;

    virtual void onMessage(twai_message_t rx_message) = 0;

protected:
    uint16_t readUnsignedInt16(uint8_t *data);
    int16_t readSignedInt16(uint8_t *data);
    float scaleValue(int16_t value, float factor);
    bool getBit(uint8_t value, uint8_t bit);

    bool _verboseLogging = true;

private:
    char* _providerName = const_cast<char*>("Battery CAN");
};
