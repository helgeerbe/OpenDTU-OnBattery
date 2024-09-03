// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022-2024 Thomas Basler and others
 */

#include "ArduinoJson.h"
#include "AsyncJson.h"
#include "Battery.h"
#include "Configuration.h"
#include "MqttHandleBatteryHass.h"
#include "MqttHandlePowerLimiterHass.h"
#include "WebApi.h"
#include "WebApi_battery.h"
#include "WebApi_errors.h"
#include "helper.h"

void WebApiBatteryClass::init(AsyncWebServer& server, Scheduler& scheduler)
{
    using std::placeholders::_1;

    _server = &server;

    _server->on("/api/battery/status", HTTP_GET, std::bind(&WebApiBatteryClass::onStatus, this, _1));
    _server->on("/api/battery/config", HTTP_GET, std::bind(&WebApiBatteryClass::onAdminGet, this, _1));
    _server->on("/api/battery/config", HTTP_POST, std::bind(&WebApiBatteryClass::onAdminPost, this, _1));
}

void WebApiBatteryClass::onStatus(AsyncWebServerRequest* request)
{
    if (!WebApi.checkCredentialsReadonly(request)) {
        return;
    }

    AsyncJsonResponse* response = new AsyncJsonResponse();
    auto& root = response->getRoot();
    const CONFIG_T& config = Configuration.get();

    root["enabled"] = config.Battery.Enabled;
    root["verbose_logging"] = config.Battery.VerboseLogging;
    root["provider"] = config.Battery.Provider;
    root["jkbms_interface"] = config.Battery.JkBmsInterface;
    root["jkbms_polling_interval"] = config.Battery.JkBmsPollingInterval;
    root["mqtt_soc_topic"] = config.Battery.MqttSocTopic;
    root["mqtt_soc_json_path"] = config.Battery.MqttSocJsonPath;
    root["mqtt_voltage_topic"] = config.Battery.MqttVoltageTopic;
    root["mqtt_voltage_json_path"] = config.Battery.MqttVoltageJsonPath;
    root["mqtt_voltage_unit"] = config.Battery.MqttVoltageUnit;
    root["zendure_device_type"] = config.Battery.ZendureDeviceType;
    root["zendure_device_serial"] = config.Battery.ZendureDeviceSerial;
    root["zendure_soc_min"] = config.Battery.ZendureMinSoC;
    root["zendure_soc_max"] = config.Battery.ZendureMaxSoC;
    root["zendure_bypass_mode"] = config.Battery.ZendureBypassMode;
    root["zendure_max_output"] = config.Battery.ZendureMaxOutput;

    response->setLength();
    request->send(response);
}

void WebApiBatteryClass::onAdminGet(AsyncWebServerRequest* request)
{
    if (!WebApi.checkCredentials(request)) {
        return;
    }

    onStatus(request);
}

void WebApiBatteryClass::onAdminPost(AsyncWebServerRequest* request)
{
    if (!WebApi.checkCredentials(request)) {
        return;
    }

    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonDocument root;
    if (!WebApi.parseRequestData(request, response, root)) {
        return;
    }

    auto& retMsg = response->getRoot();

    if (!root.containsKey("enabled") || !root.containsKey("provider")) {
        retMsg["message"] = "Values are missing!";
        retMsg["code"] = WebApiError::GenericValueMissing;
        WebApi.sendJsonResponse(request, response, __FUNCTION__, __LINE__);
        return;
    }

    CONFIG_T& config = Configuration.get();
    config.Battery.Enabled = root["enabled"].as<bool>();
    config.Battery.VerboseLogging = root["verbose_logging"].as<bool>();
    config.Battery.Provider = root["provider"].as<uint8_t>();
    config.Battery.JkBmsInterface = root["jkbms_interface"].as<uint8_t>();
    config.Battery.JkBmsPollingInterval = root["jkbms_polling_interval"].as<uint8_t>();
    strlcpy(config.Battery.MqttSocTopic, root["mqtt_soc_topic"].as<String>().c_str(), sizeof(config.Battery.MqttSocTopic));
    strlcpy(config.Battery.MqttSocJsonPath, root["mqtt_soc_json_path"].as<String>().c_str(), sizeof(config.Battery.MqttSocJsonPath));
    strlcpy(config.Battery.MqttVoltageTopic, root["mqtt_voltage_topic"].as<String>().c_str(), sizeof(config.Battery.MqttVoltageTopic));
    strlcpy(config.Battery.MqttVoltageJsonPath, root["mqtt_voltage_json_path"].as<String>().c_str(), sizeof(config.Battery.MqttVoltageJsonPath));
    config.Battery.MqttVoltageUnit = static_cast<BatteryVoltageUnit>(root["mqtt_voltage_unit"].as<uint8_t>());
    config.Battery.ZendureDeviceType = root["zendure_device_type"].as<uint8_t>();
    strlcpy(config.Battery.ZendureDeviceSerial, root["zendure_device_serial"].as<String>().c_str(), sizeof(config.Battery.ZendureDeviceSerial));
    config.Battery.ZendureMinSoC = root["zendure_soc_min"].as<uint8_t>();
    config.Battery.ZendureMaxSoC = root["zendure_soc_max"].as<uint8_t>();
    config.Battery.ZendureBypassMode = root["zendure_bypass_mode"].as<uint8_t>();
    config.Battery.ZendureMaxOutput = root["zendure_max_output"].as<uint16_t>();

    WebApi.writeConfig(retMsg);

    WebApi.sendJsonResponse(request, response, __FUNCTION__, __LINE__);

    Battery.updateSettings();
    MqttHandleBatteryHass.forceUpdate();

    // potentially make SoC thresholds auto-discoverable
    MqttHandlePowerLimiterHass.forceUpdate();
}
