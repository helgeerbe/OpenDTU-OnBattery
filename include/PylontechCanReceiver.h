// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include "Configuration.h"
#include "Battery.h"
#include <TaskSchedulerDeclarations.h>
#include <espMqttClient.h>
#include <driver/twai.h>
#include <Arduino.h>
#include <memory>

class PylontechCanReceiver : public BatteryProvider {
public:
    bool init(bool verboseLogging) final;
    void deinit() final;
    std::shared_ptr<BatteryStats> getStats() const final { return _stats; }

private:
    void loop();
    uint16_t readUnsignedInt16(uint8_t *data);
    int16_t readSignedInt16(uint8_t *data);
    float scaleValue(int16_t value, float factor);
    bool getBit(uint8_t value, uint8_t bit);

    void dummyData();

    Task _loopTask;
    bool _verboseLogging = true;
    std::shared_ptr<PylontechBatteryStats> _stats =
        std::make_shared<PylontechBatteryStats>();
};
