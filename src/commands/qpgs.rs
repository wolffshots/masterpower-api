use crate::command::{Command, Response};
use crate::error::{Result};
use bytes::BytesMut;
use serde_derive::Serialize;
use std::str::from_utf8;
use std::str::FromStr;
use log::{debug};

pub struct QPGS0;
pub struct QPGS2;
pub struct QPGS1;

impl Command for QPGS0 {
    const PROTOCOL_ID: &'static [u8] = b"QPGS0";
    const COMMAND_NAME: &'static str = "QueryDeviceGeneralStatusCurrent";

    type Request = ();
    type Response = QGPSResponse;
}

impl Command for QPGS1 {
    const PROTOCOL_ID: &'static [u8] = b"QPGS1";
    const COMMAND_NAME: &'static str = "QueryDeviceGeneralStatusPrimary";

    type Request = ();
    type Response = QGPSResponse;
}

impl Command for QPGS2 {
    const PROTOCOL_ID: &'static [u8] = b"QPGS2";
    const COMMAND_NAME: &'static str = "QueryDeviceGeneralStatusSecondary";

    type Request = ();
    type Response = QGPSResponse;
}

#[derive(Debug, PartialEq, Serialize)]
pub struct QGPSResponse {
    pub parallel_units_connected: bool,
    pub serial_number: u64,
    pub operation_mode: OperationMode,
    pub fault_code: u8, // 00 - 86
    pub ac_input_votage: f32, // Vac
    pub ac_input_frequency: f32, // Hz
    pub ac_output_voltage: f32, // Vac
    pub ac_output_frequency: f32, // Hz
    pub ac_output_apparent_power: u16, // VA
    pub ac_output_active_power: u16, // W
    pub percentage_of_nominal_output_power: u8, // %
    pub battery_voltage: f32, // Vdc
    pub battery_charging_current: u8, // Adc
    pub battery_approx_state_of_charge: u8, // %
    pub pv_input_voltage: f32, // Vdc
    pub total_charging_current: u8, // sum of Adc for all units
    pub total_ac_output_apparent_power: u16, // sum of VA for all units 
    pub total_ac_output_active_power: u16, // sum of W for all units
    pub total_percentage_of_nominal_output_power: u8, // average of % for all units
    pub inverter_status: InverterStatus,
    pub ac_output_mode: ACOutputMode, // 0 for single, 1 for parallel, other for three phase split
    pub battery_charging_source_priority: SourcePriority, // 1 solar first, 2 solar and utility, 3 solar only
    pub max_charging_current_set: u8, // Adc
    pub max_charging_current_possible: u8, // Adc
    pub max_ac_charging_current_set: u8, // Adc
    pub pv_input_current: f32, // Adc
    pub battery_discharge_current: u8, // Adc
}

#[derive(Debug, PartialEq, Serialize)]
pub struct InverterStatus {
    pub mppt_active: bool,
    pub ac_charging: bool,
    pub solar_charging: bool,
    pub battery_status: BatteryStatus, // 2 chars
    pub ac_input: bool, // 0 available, 1 not available
    pub ac_output: bool,
    pub reserved_bit: bool, 
}

#[derive(Debug, PartialEq, Serialize)]
pub enum BatteryStatus {
    BatteryChargingAndDischargingDisabledByBattery = 03,
    BatteryDisconnected = 02,
    BatteryVoltageLow = 01,
    BatteryVoltageNormal = 00,
}

#[derive(Debug, PartialEq, Serialize)]
pub enum OperationMode {
    PoweredOn, // P
    StandbyMode, // S
    LineMode, // L
    BatteryMode, // B
    FaultMode, // F
    ShutdownMode // D
}

#[derive(Debug, PartialEq, Serialize)]
pub enum ACOutputMode {
    SingleUnit = 0,
    ParallelOutput=1,
    Phase1of3PhaseOutput=2,
    Phase2of3PhaseOutput=3,
    Phase3of3PhaseOutput=4,
}

#[derive(Debug, PartialEq, Serialize)]
pub enum FaultCode {
    NoFault = 00,
    FanLockedWhileInverterOff = 01,
    OverTemperature = 02,
    BatteryVoltageTooHigh = 03,
    BatteryVoltageTooLow = 04,
    ACOutputShortCircuit = 05,
    ACOutputVoltageTooHigh = 06,
    ACOutputOverload = 07,
    InternalBusVoltageTooHigh = 08,
    InternalBusSoftStartFailed = 09,
    PVOverCurrent = 10,
    PVOverVoltage = 11,
    InternalDCConverterOverCurrent = 12,
    BatteryDischargeOverCurrent = 13,
    OverCurrent = 51,
    InternalBusVoltageTooLow = 52,
    InverterSoftStartFailed = 53,
    DCOverVoltageAtACOutput = 55,
    CurrentSensorFailed = 57,
    ACOutputVoltageTooLow = 58,
    ReverseCurrentProtectionActive = 60,
    FirmwareVersionInconsistent = 71,
    CurrentSharingFault = 72,
    CANCommunicationFault = 80,
    HostLoss = 81,
    SynchronizationLoss = 82,
    BatteryVoltageDetectedInconsistent = 83,
    ACInVoltageFrequencyInconsistent = 84,
    ACOutputCurrentImbalance = 85,
    ACOutputModeInconsistent = 86
}

#[derive(Debug, PartialEq, Serialize)]
pub enum SourcePriority {
    SolarFirst = 1,
    SolarAndUtility = 2,
    SolarOnly = 3
}

impl Response for QGPSResponse {
    fn decode(src: &mut BytesMut) -> Result<Self> {
        debug!("Input: {:?}", from_utf8(&src)?);
        let parallel_units_connected: bool = true;
        let serial_number: u64 = 0;
        let operation_mode: OperationMode = OperationMode::LineMode;
        let fault_code: u8 = 0;
        let ac_input_votage: f32 = 0.0;
        let ac_input_frequency: f32 = 0.0;
        let ac_output_voltage: f32 = 0.0;
        let ac_output_frequency: f32 = 0.0;
        let ac_output_apparent_power: u16 = 0;
        let ac_output_active_power: u16 = 0;
        let percentage_of_nominal_output_power: u8 = 0;
        let battery_voltage: f32 = 0.0;
        let battery_charging_current: u8 = 0;
        let battery_approx_state_of_charge: u8 = 0;
        let pv_input_voltage: f32 = 0.0;
        let total_charging_current: u8 = 0;
        let total_ac_output_apparent_power: u16 = 0;
        let total_ac_output_active_power: u16 = 0;
        let total_percentage_of_nominal_output_power: u8 = 0;
        let inverter_status = InverterStatus {
           mppt_active: true,
           ac_charging: true,
           solar_charging: true,
           battery_status: BatteryStatus::BatteryVoltageNormal,
           ac_input: true,
           ac_output: true,
           reserved_bit: true, 
        };
        let ac_output_mode: ACOutputMode = ACOutputMode::ParallelOutput;
        let battery_charging_source_priority: SourcePriority = SourcePriority::SolarFirst;
        let max_charging_current_set: u8 = 0;
        let max_charging_current_possible: u8 = 0;
        let max_ac_charging_current_set: u8 = 0;
        let pv_input_current: f32 = 0.0;
        let battery_discharge_current: u8 = 0;

        Ok(Self {
            parallel_units_connected,
            serial_number,
            operation_mode,
            fault_code,
            ac_input_votage,
            ac_input_frequency,
            ac_output_voltage,
            ac_output_frequency,
            ac_output_apparent_power,
            ac_output_active_power,
            percentage_of_nominal_output_power,
            battery_voltage,
            battery_charging_current,
            battery_approx_state_of_charge,
            pv_input_voltage,
            total_charging_current,
            total_ac_output_apparent_power,
            total_ac_output_active_power,
            total_percentage_of_nominal_output_power,
            inverter_status,
            ac_output_mode,
            battery_charging_source_priority,
            max_charging_current_set,
            max_charging_current_possible,
            max_ac_charging_current_set,
            pv_input_current,
            battery_discharge_current,
        })
    }
}