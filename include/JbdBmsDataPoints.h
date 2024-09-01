#pragma once

#include <Arduino.h>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <frozen/map.h>
#include <frozen/string.h>

namespace JbdBms {

#define JBD_PROTECTION_STATUS(fnc) \
    fnc(CellOverVoltage, (1<<0)) \
    fnc(CellUnderVoltage, (1<<1)) \
    fnc(PackOverVoltage, (1<<2)) \
    fnc(PackUnderVoltage, (1<<3)) \
    fnc(ChargingOverTemperature, (1<<4)) \
    fnc(ChargingLowTemperature, (1<<5)) \
    fnc(DischargingOverTemperature, (1<<6)) \
    fnc(DischargingLowTemperature, (1<<7)) \
    fnc(ChargingOverCurrent, (1<<8)) \
    fnc(DischargeOverCurrent, (1<<9)) \
    fnc(ShortCircuit, (1<<10)) \
    fnc(IcFrontEndError, (1<<11)) \
    fnc(MosSotwareLock, (1<<12)) \
    fnc(Reserved1, (1<<13)) \
    fnc(Reserved2, (1<<14)) \
    fnc(Reserved3, (1<<15))

enum class AlarmBits : uint16_t {
#define ALARM_ENUM(name, value) name = value,
    JBD_PROTECTION_STATUS(ALARM_ENUM)
#undef ALARM_ENUM
};

static const frozen::map<AlarmBits, frozen::string, 16> AlarmBitTexts = {
#define ALARM_TEXT(name, value) { AlarmBits::name, #name },
    JBD_PROTECTION_STATUS(ALARM_TEXT)
#undef ALARM_TEXT
};

enum class DataPointLabel : uint8_t {
    CellsMilliVolt,
    BatteryTempOneCelsius,
    BatteryTempTwoCelsius,
    BatteryVoltageMilliVolt,
    BatteryCurrentMilliAmps,
    BatterySoCPercent,
    BatteryTemperatureSensorAmount,
    BatteryCycles,
    BatteryCellAmount,
    AlarmsBitmask,
    BalancingEnabled,
    CellAmountSetting,
    BatteryCapacitySettingAmpHours,
    BatteryChargeEnabled,
    BatteryDischargeEnabled,
    DateOfManufacturing,
    BmsSoftwareVersion,
    BmsHardwareVersion,
    ActualBatteryCapacityAmpHours
};

using tCells = std::map<uint8_t, uint16_t>;

template<DataPointLabel> struct DataPointLabelTraits;

#define LABEL_TRAIT(n, t, u) template<> struct DataPointLabelTraits<DataPointLabel::n> { \
    using type = t; \
    static constexpr char const name[] = #n; \
    static constexpr char const unit[] = u; \
};

/**
 * the types associated with the labels are the types for the respective data
 * points in the JbdBms::DataPoint class. they are *not* always equal to the
 * type used in the serial message.
 *
 * it is unfortunate that we have to repeat all enum values here to define the
 * traits. code generation could help here (labels are defined in a single
 * source of truth and this code is generated -- no typing errors, etc.).
 * however, the compiler will complain if an enum is misspelled or traits are
 * defined for a removed enum, so we will notice. it will also complain when a
 * trait is missing and if a data point for a label without traits is added to
 * the DataPointContainer class, because the traits must be available then.
 * even though this is tedious to maintain, human errors will be caught.
 */
LABEL_TRAIT(CellsMilliVolt,                         tCells,      "mV");
LABEL_TRAIT(BatteryTempOneCelsius,                  int16_t,     "°C");
LABEL_TRAIT(BatteryTempTwoCelsius,                  int16_t,     "°C");
LABEL_TRAIT(BatteryVoltageMilliVolt,                uint32_t,    "mV");
LABEL_TRAIT(BatteryCurrentMilliAmps,                int32_t,     "mA");
LABEL_TRAIT(BatterySoCPercent,                      uint8_t,     "%");
LABEL_TRAIT(BatteryTemperatureSensorAmount,         uint8_t,     "");
LABEL_TRAIT(BatteryCycles,                          uint16_t,    "");
LABEL_TRAIT(BatteryCellAmount,                      uint16_t,    "");
LABEL_TRAIT(AlarmsBitmask,                          uint16_t,    "");
LABEL_TRAIT(BalancingEnabled,                       bool,        "");
LABEL_TRAIT(CellAmountSetting,                      uint8_t,     "");
LABEL_TRAIT(BatteryCapacitySettingAmpHours,         uint32_t,    "Ah");
LABEL_TRAIT(BatteryChargeEnabled,                   bool,        "");
LABEL_TRAIT(BatteryDischargeEnabled,                bool,        "");
LABEL_TRAIT(DateOfManufacturing,                    std::string, "");
LABEL_TRAIT(BmsSoftwareVersion,                     std::string, "");
LABEL_TRAIT(BmsHardwareVersion,                     std::string, "");
LABEL_TRAIT(ActualBatteryCapacityAmpHours,          uint32_t,    "Ah");
#undef LABEL_TRAIT

class DataPoint {
    friend class DataPointContainer;

    public:
        using tValue = std::variant<bool, uint8_t, uint16_t, uint32_t,
              int16_t, int32_t, std::string, tCells>;

        DataPoint() = delete;

        DataPoint(DataPoint const& other)
            : _strLabel(other._strLabel)
            , _strValue(other._strValue)
            , _strUnit(other._strUnit)
            , _value(other._value)
            , _timestamp(other._timestamp) { }

        DataPoint(std::string const& strLabel, std::string const& strValue,
                std::string const& strUnit, tValue value, uint32_t timestamp)
            : _strLabel(strLabel)
            , _strValue(strValue)
            , _strUnit(strUnit)
            , _value(std::move(value))
            , _timestamp(timestamp) { }

        std::string const& getLabelText() const { return _strLabel; }
        std::string const& getValueText() const { return _strValue; }
        std::string const& getUnitText() const { return _strUnit; }
        uint32_t getTimestamp() const { return _timestamp; }

        bool operator==(DataPoint const& other) const {
            return _value == other._value;
        }

    private:
        std::string _strLabel;
        std::string _strValue;
        std::string _strUnit;
        tValue _value;
        uint32_t _timestamp;
};

template<typename T> std::string dataPointValueToStr(T const& v);

class DataPointContainer {
    public:
        DataPointContainer() = default;

        using Label = DataPointLabel;
        template<Label L> using Traits = JbdBms::DataPointLabelTraits<L>;

        template<Label L>
        void add(typename Traits<L>::type val) {
            _dataPoints.emplace(
                    L,
                    DataPoint(
                        Traits<L>::name,
                        dataPointValueToStr(val),
                        Traits<L>::unit,
                        DataPoint::tValue(std::move(val)),
                        millis()
                    )
            );
        }

        // make sure add() is only called with the type expected for the
        // respective label, no implicit conversions allowed.
        template<Label L, typename T>
        void add(T) = delete;

        template<Label L>
        std::optional<DataPoint const> getDataPointFor() const {
            auto it = _dataPoints.find(L);
            if (it == _dataPoints.end()) { return std::nullopt; }
            return it->second;
        }

        template<Label L>
        std::optional<typename Traits<L>::type> get() const {
            auto optionalDataPoint = getDataPointFor<L>();
            if (!optionalDataPoint.has_value()) { return std::nullopt; }
            return std::get<typename Traits<L>::type>(optionalDataPoint->_value);
        }

        using tMap = std::unordered_map<Label, DataPoint const>;
        tMap::const_iterator cbegin() const { return _dataPoints.cbegin(); }
        tMap::const_iterator cend() const { return _dataPoints.cend(); }

        // copy all data points from source into this instance, overwriting
        // existing data points in this instance.
        void updateFrom(DataPointContainer const& source);

    private:
        tMap _dataPoints;
};

} /* namespace JbdBms */
