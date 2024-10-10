export interface BatteryConfig {
    enabled: boolean;
    verbose_logging: boolean;
    provider: number;
    jkbms_interface: number;
    jkbms_polling_interval: number;
    can_interface: number;
    mqtt_can_topic: string;
    mqtt_soc_topic: string;
    mqtt_soc_json_path: string;
    mqtt_voltage_topic: string;
    mqtt_voltage_json_path: string;
    mqtt_voltage_unit: number;
    enable_discharge_current_limit: boolean;
    discharge_current_limit: number;
    use_battery_reported_discharge_current_limit: boolean;
    mqtt_discharge_current_topic: string;
    mqtt_discharge_current_json_path: string;
    mqtt_amperage_unit: number;
}
