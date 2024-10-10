#pragma once

#include <optional>
#include "Battery.h"
#include "MessageOutput.h"
#include <espMqttClient.h>

// DeviceIDs for compatible Solarflow devices
#define ZENDURE_HUB1200     "73bkTV"
#define ZENDURE_HUB2000     "A8yh63"
#define ZENDURE_AIO2400     "yWF7hV)"
#define ZENDURE_ACE1500     "8bM93H"
#define ZENDURE_HYPER2000   "ja72U0ha)"

#define ZENDURE_MAX_PACKS                           4U
#define ZENDURE_REMAINING_TIME_OVERFLOW             59940U

#define ZENDURE_SECONDS_SUNPOSITION                 60U
#define ZENDURE_SECONDS_TIMESYNC                    3600U

#define ZENDURE_LOG_ROOT                            "log"
#define ZENDURE_LOG_SERIAL                          "sn"
#define ZENDURE_LOG_PARAMS                          "params"

#define ZENDURE_LOG_OFFSET_SOC                      0U                  // [%]
#define ZENDURE_LOG_OFFSET_PACKNUM                  1U                  // [1]
#define ZENDURE_LOG_OFFSET_PACK_SOC(pack)           (2U+(pack)-1U)      // [d%]
#define ZENDURE_LOG_OFFSET_PACK_VOLTAGE(pack)       (6U+(pack)-1U)      // [cV]
#define ZENDURE_LOG_OFFSET_PACK_CURRENT(pack)       (10U+(pack)-1U)     // [dA]
#define ZENDURE_LOG_OFFSET_PACK_CELL_MIN(pack)      (14U+(pack)-1U)     // [cV]
#define ZENDURE_LOG_OFFSET_PACK_CELL_MAX(pack)      (18U+(pack)-1U)     // [cV]
#define ZENDURE_LOG_OFFSET_PACK_UNKOWN_1(pack)      (22U+(pack)-1U)     // ? => always (0 | 0 | 0 | 0)
#define ZENDURE_LOG_OFFSET_PACK_UNKOWN_2(pack)      (26U+(pack)-1U)     // ? => always (0 | 0 | 0 | 0)
#define ZENDURE_LOG_OFFSET_PACK_UNKOWN_3(pack)      (30U+(pack)-1U)     // ? => always (8449 | 257 | 0 | 0)
#define ZENDURE_LOG_OFFSET_PACK_TEMPERATURE(pack)   (34U+(pack)-1U)     // [°C]
#define ZENDURE_LOG_OFFSET_PACK_UNKOWN_5(pack)      (38U+(pack)-1U)     // ? => always (1340 | 99 | 0 | 0)
#define ZENDURE_LOG_OFFSET_VOLTAGE                  42U                 // [dV]
#define ZENDURE_LOG_OFFSET_SOLAR_POWER_MPPT_2       43U                 // [W]
#define ZENDURE_LOG_OFFSET_SOLAR_POWER_MPPT_1       44U                 // [W]
#define ZENDURE_LOG_OFFSET_OUTPUT_POWER             45U                 // [W]
#define ZENDURE_LOG_OFFSET_UNKOWN_05                46U                 // ? => 1, 413
#define ZENDURE_LOG_OFFSET_DISCHARGE_POWER          47U                 // [W]
#define ZENDURE_LOG_OFFSET_CHARGE_POWER             48U                 // [W]
#define ZENDURE_LOG_OFFSET_OUTPUT_POWER_LIMIT       49U                 // [cA]
#define ZENDURE_LOG_OFFSET_UNKOWN_08                50U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_09                51U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_10                52U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_11                53U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_12                54U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_BYPASS_MODE              55U                 // [0=Auto | 1=AlwaysOff | 2=AlwaysOn]
#define ZENDURE_LOG_OFFSET_UNKOWN_14                56U                 // ? => always 3
#define ZENDURE_LOG_OFFSET_UNKOWN_15                57U                 // ? Some kind of bitmask => e.g. 813969441
#define ZENDURE_LOG_OFFSET_UNKOWN_16                58U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_17                59U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_18                60U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_19                61U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_20                62U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_21                63U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_22                64U                 // ? => always 1
#define ZENDURE_LOG_OFFSET_UNKOWN_23                65U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_24                66U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_25                67U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_26                68U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_27                69U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_28                70U                 // ? => always 1
#define ZENDURE_LOG_OFFSET_UNKOWN_29                71U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_30                72U                 // ? some counter => 258, 263, 25
#define ZENDURE_LOG_OFFSET_UNKOWN_31                73U                 // ? some counter => 309, 293, 23
#define ZENDURE_LOG_OFFSET_UNKOWN_32                74U                 // ? => always 1
#define ZENDURE_LOG_OFFSET_UNKOWN_33                75U                 // ? => always 1
#define ZENDURE_LOG_OFFSET_UNKOWN_34                76U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_35                77U                 // ? => always 0 or 1
#define ZENDURE_LOG_OFFSET_UNKOWN_36                78U                 // ? => always 0 or 1
#define ZENDURE_LOG_OFFSET_UNKOWN_37                79U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_38                80U                 // ? => always 1 or 0
#define ZENDURE_LOG_OFFSET_AUTO_RECOVER             81U                 // [bool]
#define ZENDURE_LOG_OFFSET_UNKOWN_40                82U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_41                83U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_42                84U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_MIN_SOC                  85U                 // [%]
#define ZENDURE_LOG_OFFSET_UNKOWN_44                86U                 // State 0 == Idle | 1 == Discharge
#define ZENDURE_LOG_OFFSET_UNKOWN_45                87U                 // ? => always 512
#define ZENDURE_LOG_OFFSET_UNKOWN_46                88U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_47                89U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_48                90U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_49                91U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_50                92U                 // ? => always 0
#define ZENDURE_LOG_OFFSET_UNKOWN_51                93U                 // ? => always 0

#define ZENDURE_REPORT_PROPERTIES                   "properties"
#define ZENDURE_REPORT_MIN_SOC                      "minSoc"
#define ZENDURE_REPORT_MAX_SOC                      "socSet"
#define ZENDURE_REPORT_INPUT_LIMIT                  "inputLimit"
#define ZENDURE_REPORT_OUTPUT_LIMIT                 "outputLimit"
#define ZENDURE_REPORT_INVERSE_MAX_POWER            "inverseMaxPower"
#define ZENDURE_REPORT_HEAT_STATE                   "heatState"
#define ZENDURE_REPORT_AUTO_SHUTDOWN                "hubState"
#define ZENDURE_REPORT_BUZZER_SWITCH                "buzzerSwitch"
#define ZENDURE_REPORT_REMAIN_OUT_TIME              "remainOutTime"
#define ZENDURE_REPORT_REMAIN_IN_TIME               "remainInputTime"
#define ZENDURE_REPORT_MASTER_FW_VERSION            "masterSoftVersion"
#define ZENDURE_REPORT_MASTER_HW_VERSION            "masterhaerVersion"
#define ZENDURE_REPORT_HUB_STATE                    "state"
#define ZENDURE_REPORT_BATTERY_STATE                "packState"
#define ZENDURE_REPORT_AUTO_RECOVER                 "autoRecover"
#define ZENDURE_REPORT_BYPASS_STATE                 "pass"
#define ZENDURE_REPORT_BYPASS_MODE                  "passMode"
#define ZENDURE_REPORT_PV_BRAND                     "pvBrand"
#define ZENDURE_REPORT_PV_AUTO_MODEL                "autoModel"
#define ZENDURE_REPORT_MASTER_SWITCH                "masterSwitch"
#define ZENDURE_REPORT_AC_MODE                      "acMode"
#define ZENDURE_REPORT_INPUT_MODE                   "inputMode"

#define ZENDURE_REPORT_SOLAR_POWER_MPPT(x)          "solarPower"##x
#define ZENDURE_REPORT_SOLAR_INPUT_POWER            "solarInputPower"
#define ZENDURE_REPORT_GRID_INPUT_POWER             "gridInputPower"    // Hyper2000/Ace1500 only - need to check
#define ZENDURE_REPORT_CHARGE_POWER                 "packInputPower"
#define ZENDURE_REPORT_DISCHARGE_POWER              "outputPackPower"
#define ZENDURE_REPORT_OUTPUT_POWER                 "outputHomePower"
#define ZENDURE_REPORT_DC_OUTPUT_POWER              "dcOutputPower"     // Ace1500 only - need to check
#define ZENDURE_REPORT_AC_OUTPUT_POWER              "acOutputPower"     // Hyper2000 only - need to check

#define ZENDURE_REPORT_SMART_MODE                   "smartMode"
#define ZENDURE_REPORT_SMART_POWER                  "smartPower"
#define ZENDURE_REPORT_GRID_POWER                   "gridPower"
#define ZENDURE_REPORT_BLUE_OTA                     "blueOta"
#define ZENDURE_REPORT_WIFI_STATE                   "wifiState"
#define ZENDURE_REPORT_AC_SWITCH                    "acSwitch"          // Hyper2000/Ace1500 only - need to check
#define ZENDURE_REPORT_DC_SWITCH                    "dcSwitch"          // Ace1500 only - need to check




#define ZENDURE_REPORT_PACK_DATE                    "packData"
#define ZENDURE_REPORT_PACK_SERIAL                  "sn"
#define ZENDURE_REPORT_PACK_STATE                   "state"
#define ZENDURE_REPORT_PACK_POWER                   "power"
#define ZENDURE_REPORT_PACK_SOC                     "socLevel"
#define ZENDURE_REPORT_PACK_CELL_MAX_TEMPERATURE    "maxTemp"
#define ZENDURE_REPORT_PACK_CELL_MIN_VOLATAGE       "minVol"
#define ZENDURE_REPORT_PACK_CELL_MAX_VOLATAGE       "maxVol"
#define ZENDURE_REPORT_PACK_TOTAL_VOLATAGE          "totalVol"
#define ZENDURE_REPORT_PACK_FW_VERSION              "softVersion"
#define ZENDURE_REPORT_PACK_HEALTH                  "soh"

#define ZENDURE_ALIVE_SECONDS                       ( 5 * 60 )
#define ZENDURE_NO_REDUCED_UPDATE

#define ZENDURE_PERSISTENT_SETTINGS_LAST_FULL      "lastFullEpoch"
#define ZENDURE_PERSISTENT_SETTINGS_LAST_EMPTY     "lastEmptyEpoch"
#define ZENDURE_PERSISTENT_SETTINGS_CHARGE_THROUGH "chargeThrough"
#define ZENDURE_PERSISTENT_SETTINGS                {ZENDURE_PERSISTENT_SETTINGS_LAST_FULL, ZENDURE_PERSISTENT_SETTINGS_LAST_EMPTY, ZENDURE_PERSISTENT_SETTINGS_CHARGE_THROUGH}

class ZendureBattery : public BatteryProvider {
public:
    ZendureBattery() = default;

    bool init(bool verboseLogging) final;
    void deinit() final;
    void loop() final;
    std::shared_ptr<BatteryStats> getStats() const final { return _stats; }

    uint16_t setOutputLimit(uint16_t limit) const;
    uint16_t setInverterMax(uint16_t limit) const;
    void shutdown() const;

    bool checkChargeThrough(uint32_t predictHours = 0U);

protected:
    void timesync();
    static String parseVersion(uint32_t version);
    uint16_t calcOutputLimit(uint16_t limit) const;
    void setTargetSoCs(const float soc_min, const float soc_max);

private:
    void calculateEfficiency();
    void calculateFullChargeAge();
    void publishProperty(const String& topic, const String& property, const String& value) const;
    template<typename... Arg>
    void publishProperties(const String& topic, Arg&&... args) const;

    void setSoC(const float soc, const uint32_t timestamp = 0, const uint8_t precision = 2);
    bool setChargeThrough(const bool value, const bool publish = true);

    void rescheduleSunCalc() { _nextSunCalc = 0; }
    bool alive() const { return _stats->getAgeSeconds() < ZENDURE_ALIVE_SECONDS; }

    void publishPersistentSettings(const char* subtopic, const String& payload);

    template <typename... Args>
    void log(char const* format, Args&&... args) const {
        if (_verboseLogging) {
            MessageOutput.printf("ZendureBattery: ");
            MessageOutput.printf(format, std::forward<Args>(args)...);
            MessageOutput.println();
        }
        return;
    };

    bool _verboseLogging = false;

#ifndef ZENDURE_NO_REDUCED_UPDATE
    uint32_t _rateUpdateMs;
    uint64_t _nextUpdate;
#endif

    uint32_t _rateFullUpdateMs = 0;
    uint64_t _nextFullUpdate = 0;

    uint32_t _rateTimesyncMs = 0;
    uint64_t _nextTimesync = 0;

    uint32_t _rateSunCalcMs = 0;
    uint64_t _nextSunCalc = 0;

    uint32_t _messageCounter = 0;

    String _deviceId = String();

    String _baseTopic = String();
    String _topicLog = String();
    String _topicReadReply = String();
    String _topicReport = String();
    String _topicRead = String();
    String _topicWrite = String();
    String _topicTimesync = String();
    String _topicPersistentSettings = String();

    String _payloadSettings = String();
    String _payloadFullUpdate = String();
#ifndef ZENDURE_NO_REDUCED_UPDATE
    String _payloadUpdate;
#endif


    std::shared_ptr<ZendureBatteryStats> _stats = std::make_shared<ZendureBatteryStats>();

    void onMqttMessageReport(espMqttClientTypes::MessageProperties const& properties,
            char const* topic, uint8_t const* payload, size_t len, size_t index, size_t total);

    void onMqttMessageLog(espMqttClientTypes::MessageProperties const& properties,
            char const* topic, uint8_t const* payload, size_t len, size_t index, size_t total);

    void onMqttMessageTimesync(espMqttClientTypes::MessageProperties const& properties,
            char const* topic, uint8_t const* payload, size_t len, size_t index, size_t total);

    void onMqttMessagePersistentSettings(espMqttClientTypes::MessageProperties const& properties,
            char const* topic, uint8_t const* payload, size_t len, size_t index, size_t total);

#ifndef ZENDURE_NO_REDUCED_UPDATE
    void onMqttMessageRead(espMqttClientTypes::MessageProperties const& properties,
            char const* topic, uint8_t const* payload, size_t len, size_t index, size_t total);
#endif
};