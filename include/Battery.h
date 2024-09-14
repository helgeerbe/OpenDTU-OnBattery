// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <memory>
#include <mutex>
#include <TaskSchedulerDeclarations.h>

#include "BatteryStats.h"

class BatteryProvider {
public:
    // returns true if the provider is ready for use, false otherwise
    virtual bool init(bool verboseLogging) = 0;
    virtual void deinit() = 0;
    virtual void loop() = 0;
    virtual std::shared_ptr<BatteryStats> getStats() const = 0;
};

class BatteryClass {
public:
    void init(Scheduler&);
    void updateSettings();

    float getDischargeCurrentLimit();

    std::shared_ptr<BatteryStats const> getStats() const;

private:
    void loop();

    Task _loopTask;
    mutable std::mutex _mutex;
    std::unique_ptr<BatteryProvider> _upProvider = nullptr;
};

extern BatteryClass Battery;
