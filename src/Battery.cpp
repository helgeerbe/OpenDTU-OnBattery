// SPDX-License-Identifier: GPL-2.0-or-later
#include "Battery.h"
#include "MessageOutput.h"
#include "MqttSettings.h"
#include "PylontechCanReceiver.h"
#include "JkBmsController.h"
#include "VictronSmartShunt.h"
#include "MqttBattery.h"
#include "Scheduler.h"

BatteryClass Battery;

std::shared_ptr<BatteryStats const> BatteryClass::getStats() const
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (!_upProvider) {
        static auto sspDummyStats = std::make_shared<BatteryStats>();
        return sspDummyStats;
    }

    return _upProvider->getStats();
}

void BatteryClass::init(Scheduler& s)
{
    MessageOutput.printf("scheduler = %p, s = %p\r\n", &scheduler, &s);
    scheduler.addTask(_mqttPublishTask);
    _mqttPublishTask.setCallback(std::bind(&BatteryClass::mqttPublish, this));
    _mqttPublishTask.setIterations(TASK_FOREVER);
    _mqttPublishTask.enable();

    this->updateSettings();
}

void BatteryClass::updateSettings()
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (_upProvider) {
        _upProvider->deinit();
        _upProvider = nullptr;
        _mqttPublishTask.disable();
        MessageOutput.println("disabled _mqttPublish task");
    }

    auto const& config = Configuration.get();
    if (!config.Battery.Enabled) { return; }

    bool verboseLogging = config.Battery.VerboseLogging;

    switch (config.Battery.Provider) {
        case 0:
            _upProvider = std::make_unique<PylontechCanReceiver>();
            if (!_upProvider->init(verboseLogging)) { _upProvider = nullptr; }
            break;
        case 1:
            _upProvider = std::make_unique<JkBms::Controller>();
            if (!_upProvider->init(verboseLogging)) { _upProvider = nullptr; }
            break;
        case 2:
            _upProvider = std::make_unique<MqttBattery>();
            if (!_upProvider->init(verboseLogging)) { _upProvider = nullptr; }
            break;
        case 3:
            _upProvider = std::make_unique<VictronSmartShunt>();
            if (!_upProvider->init(verboseLogging)) { _upProvider = nullptr; }
            break;
        default:
            MessageOutput.printf("Unknown battery provider: %d\r\n", config.Battery.Provider);
            break;
    }

    if (_upProvider) {
        _mqttPublishTask.setInterval(config.Mqtt.PublishInterval * TASK_SECOND);
        _mqttPublishTask.enable();
        MessageOutput.println("re-enabled _mqttPublish task");
    }
}

void BatteryClass::mqttPublish()
{
    MessageOutput.println("this is BatteryClass::mqttPublish()");
    std::unique_lock<std::mutex> lock(_mutex);

    if (!_upProvider || !MqttSettings.getConnected()) { return; }

    auto spStats = _upProvider->getStats();

    lock.unlock();

    spStats->mqttPublish();
}
