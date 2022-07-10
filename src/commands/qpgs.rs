use crate::command::{Command, Response};
use crate::error::Result;
use bytes::BytesMut;
use log::debug;
use serde_derive::Serialize;
use std::str::from_utf8;
use std::str::FromStr;

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
    pub other_units_connected: bool,
    pub serial_number: u64,
    pub operation_mode: OperationMode,
    pub fault_code: FaultCode,                        // 00 - 86
    pub ac_input_votage: f32,                         // Vac
    pub ac_input_frequency: f32,                      // Hz
    pub ac_output_voltage: f32,                       // Vac
    pub ac_output_frequency: f32,                     // Hz
    pub ac_output_apparent_power: u16,                // VA
    pub ac_output_active_power: u16,                  // W
    pub percentage_of_nominal_output_power: u8,       // %
    pub battery_voltage: f32,                         // Vdc
    pub battery_charging_current: u8,                 // Adc
    pub battery_approx_state_of_charge: u8,           // %
    pub pv_input_voltage: f32,                        // Vdc
    pub total_charging_current: u8,                   // sum of Adc for all units
    pub total_ac_output_apparent_power: u16,          // sum of VA for all units
    pub total_ac_output_active_power: u16,            // sum of W for all units
    pub total_percentage_of_nominal_output_power: u8, // average of % for all units
    pub inverter_status: InverterStatus,
    pub ac_output_mode: ACOutputMode, // 0 for single, 1 for parallel, other for three phase split
    pub battery_charging_source_priority: SourcePriority, // 1 solar first, 2 solar and utility, 3 solar only
    pub max_charging_current_set: u8,                     // Adc
    pub max_charging_current_possible: u8,                // Adc
    pub max_ac_charging_current_set: u8,                  // Adc
    pub pv_input_current: f32,                            // Adc
    pub battery_discharge_current: u8,                    // Adc
}

#[derive(Debug, PartialEq, Serialize)]
pub struct InverterStatus {
    pub mppt_active: State,
    pub ac_charging: State,
    pub solar_charging: State,
    pub battery_status: BatteryStatus, // 2 chars
    pub ac_input: State,                // 0 available, 1 not available
    pub ac_output: State,
    pub reserved_bit: State,
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
    PoweredOn,    // P
    StandbyMode,  // S
    LineMode,     // L
    BatteryMode,  // B
    FaultMode,    // F
    ShutdownMode, // D
}

#[derive(Debug, PartialEq, Serialize)]
pub enum ACOutputMode {
    SingleUnit = 0,
    ParallelOutput = 1,
    Phase1of3PhaseOutput = 2,
    Phase2of3PhaseOutput = 3,
    Phase3of3PhaseOutput = 4,
}

#[derive(Debug, PartialEq, Serialize, Copy, Clone)]
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
    ACOutputModeInconsistent = 86,
}

fn u8_to_fault_code(fault_code: u8) -> FaultCode {
    match fault_code {
        00 => FaultCode::NoFault,
        01 => FaultCode::FanLockedWhileInverterOff,
        02 => FaultCode::OverTemperature,
        03 => FaultCode::BatteryVoltageTooHigh,
        04 => FaultCode::BatteryVoltageTooLow,
        05 => FaultCode::ACOutputShortCircuit,
        06 => FaultCode::ACOutputVoltageTooHigh,
        07 => FaultCode::ACOutputOverload,
        08 => FaultCode::InternalBusVoltageTooHigh,
        09 => FaultCode::InternalBusSoftStartFailed,
        10 => FaultCode::PVOverCurrent,
        11 => FaultCode::PVOverVoltage,
        12 => FaultCode::InternalDCConverterOverCurrent,
        13 => FaultCode::BatteryDischargeOverCurrent,
        51 => FaultCode::OverCurrent,
        52 => FaultCode::InternalBusVoltageTooLow,
        53 => FaultCode::InverterSoftStartFailed,
        55 => FaultCode::DCOverVoltageAtACOutput,
        57 => FaultCode::CurrentSensorFailed,
        58 => FaultCode::ACOutputVoltageTooLow,
        60 => FaultCode::ReverseCurrentProtectionActive,
        71 => FaultCode::FirmwareVersionInconsistent,
        72 => FaultCode::CurrentSharingFault,
        80 => FaultCode::CANCommunicationFault,
        81 => FaultCode::HostLoss,
        82 => FaultCode::SynchronizationLoss,
        83 => FaultCode::BatteryVoltageDetectedInconsistent,
        84 => FaultCode::ACInVoltageFrequencyInconsistent,
        85 => FaultCode::ACOutputCurrentImbalance,
        86 => FaultCode::ACOutputModeInconsistent,
        _ => unreachable!(),
    }
}

#[test]
fn test_enum_matches_values() -> Result<()> {
    assert_eq!(FaultCode::ACOutputModeInconsistent as u8, 86);
    assert_eq!(FaultCode::OverCurrent, u8_to_fault_code(51));
    Ok(())
}

#[derive(Debug, PartialEq, Serialize)]
pub enum SourcePriority {
    SolarFirst = 1,
    SolarAndUtility = 2,
    SolarOnly = 3,
}

#[derive(Debug, PartialEq, Serialize)]
pub enum State {
    Active,
    Inactive
}

impl Response for QGPSResponse {
    fn decode(src: &mut BytesMut) -> Result<Self> {
        debug!("Input: {:?}", from_utf8(&src)?);

        let mut idxs = [0usize; 26]; // there are 27 entries but 26 indices
        debug!("idxs before: {:?}", idxs);
        src.iter()
            .cloned()
            .enumerate()
            .filter(|(_, x)| *x == ' ' as u8)
            .fold(0, |num_idx, byte_idx| {
                idxs[num_idx] = byte_idx.0;
                num_idx + 1
            });
        debug!("idxs after: {:?}", idxs);
        let other_units_connected: bool = from_utf8(&src[0..idxs[0]])? == "1";
        let serial_number: u64 = u64::from_str(from_utf8(&src[idxs[0] + 1..idxs[1]])?)?;
        debug!("serial number: {:?}", serial_number);
        let operation_mode: OperationMode = match from_utf8(&src[idxs[1] + 1..idxs[2]])? {
            "P" => OperationMode::PoweredOn,    // P
            "S" => OperationMode::StandbyMode,  // S
            "L" => OperationMode::LineMode,     // L
            "B" => OperationMode::BatteryMode,  // B
            "F" => OperationMode::FaultMode,    // F
            "D" => OperationMode::ShutdownMode, // D
            _ => unreachable!(),
        };
        let fault_code = u8::from_str(from_utf8(&src[idxs[2] + 1..idxs[3]])?)?;
        let fault_code: FaultCode = u8_to_fault_code(fault_code);
        let ac_input_votage: f32 = f32::from_str(from_utf8(&src[idxs[3] + 1..idxs[4]])?)?;
        let ac_input_frequency: f32 = f32::from_str(from_utf8(&src[idxs[4] + 1..idxs[5]])?)?;
        let ac_output_voltage: f32 = f32::from_str(from_utf8(&src[idxs[5] + 1..idxs[6]])?)?;
        let ac_output_frequency: f32 = f32::from_str(from_utf8(&src[idxs[6] + 1..idxs[7]])?)?;
        let ac_output_apparent_power: u16 = u16::from_str(from_utf8(&src[idxs[7] + 1..idxs[8]])?)?;
        let ac_output_active_power: u16 = u16::from_str(from_utf8(&src[idxs[8] + 1..idxs[9]])?)?;
        let percentage_of_nominal_output_power: u8 =
            u8::from_str(from_utf8(&src[idxs[9] + 1..idxs[10]])?)?;
        let battery_voltage: f32 = f32::from_str(from_utf8(&src[idxs[10] + 1..idxs[11]])?)?;
        let battery_charging_current: u8 = u8::from_str(from_utf8(&src[idxs[11] + 1..idxs[12]])?)?;
        let battery_approx_state_of_charge: u8 =
            u8::from_str(from_utf8(&src[idxs[12] + 1..idxs[13]])?)?;
        let pv_input_voltage: f32 = f32::from_str(from_utf8(&src[idxs[13] + 1..idxs[14]])?)?;
        let total_charging_current: u8 = u8::from_str(from_utf8(&src[idxs[14] + 1..idxs[15]])?)?;
        let total_ac_output_apparent_power: u16 =
            u16::from_str(from_utf8(&src[idxs[15] + 1..idxs[16]])?)?;
        let total_ac_output_active_power: u16 =
            u16::from_str(from_utf8(&src[idxs[16] + 1..idxs[17]])?)?;
        let total_percentage_of_nominal_output_power: u8 =
            u8::from_str(from_utf8(&src[idxs[17] + 1..idxs[18]])?)?;
        let inverter_status = from_utf8(&src[idxs[18] + 1..idxs[19]])?;
        assert_eq!(inverter_status.len(), 8);
        let inverter_status = InverterStatus {
            mppt_active: match &inverter_status[0..1] {
                "1" => State::Active,
                "0" => State::Inactive,
                _ => unreachable!()
            },
            ac_charging: match &inverter_status[1..2] {
                "1" => State::Active,
                "0" => State::Inactive,
                _ => unreachable!()
            },
            solar_charging: match &inverter_status[2..3] {
                "1" => State::Active,
                "0" => State::Inactive,
                _ => unreachable!()
            },
            battery_status: match &inverter_status[3..5] {
                "00" => BatteryStatus::BatteryVoltageNormal,
                "01" => BatteryStatus::BatteryVoltageLow,
                "02" => BatteryStatus::BatteryDisconnected,
                "03" => BatteryStatus::BatteryChargingAndDischargingDisabledByBattery,
                _ => unreachable!()
            },
            ac_input: match &inverter_status[5..6] {
                "1" => State::Active,
                "0" => State::Inactive,
                _ => unreachable!()
            },
            ac_output: match &inverter_status[6..7] {
                "1" => State::Active,
                "0" => State::Inactive,
                _ => unreachable!()
            },
            reserved_bit: match &inverter_status[7..] {
                "1" => State::Active,
                "0" => State::Inactive,
                _ => unreachable!()
            },
        };
        let ac_output_mode: ACOutputMode =
            match u8::from_str(from_utf8(&src[idxs[19] + 1..idxs[20]])?)? {
                0 => ACOutputMode::SingleUnit,
                1 => ACOutputMode::ParallelOutput,
                2 => ACOutputMode::Phase1of3PhaseOutput,
                3 => ACOutputMode::Phase2of3PhaseOutput,
                4 => ACOutputMode::Phase3of3PhaseOutput,
                _ => unreachable!(),
            };
        let battery_charging_source_priority: SourcePriority =
            match u8::from_str(from_utf8(&src[idxs[20] + 1..idxs[21]])?)? {
                1 => SourcePriority::SolarFirst,
                2 => SourcePriority::SolarAndUtility,
                3 => SourcePriority::SolarOnly,
                _ => unreachable!(),
            };
        let max_charging_current_set: u8 = u8::from_str(from_utf8(&src[idxs[21] + 1..idxs[22]])?)?;
        let max_charging_current_possible: u8 =
            u8::from_str(from_utf8(&src[idxs[22] + 1..idxs[23]])?)?;
        let max_ac_charging_current_set: u8 =
            u8::from_str(from_utf8(&src[idxs[23] + 1..idxs[24]])?)?;
        let pv_input_current: f32 = f32::from_str(from_utf8(&src[idxs[24] + 1..idxs[25]])?)?;
        let battery_discharge_current: u8 = u8::from_str(from_utf8(&src[idxs[25] + 1..])?)?;

        Ok(Self {
            other_units_connected,
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
