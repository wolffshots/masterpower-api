use crate::command::{Command, Response};
use crate::commands::qpigs::DeviceChargingStatus::{
    ChargingFromAC, ChargingFromSCC, ChargingFromSCCAndAC, NotCharging,
};
use crate::error::{Error, Result};
use bytes::BytesMut;
use serde_derive::Serialize;
use std::str::from_utf8;
use std::str::FromStr;
use log::{debug};

pub struct QPGS0;

impl Command for QPGS0 {
    const PROTOCOL_ID: &'static [u8] = b"QPGS0";
    const COMMAND_NAME: &'static str = "QuerySpecificDeviceGeneralStatus";

    type Request = ();
    type Response = QPGS0Response;
}

#[derive(Debug, PartialEq, Serialize)]
pub struct QPGS0Response {
    pub src: str,
}

impl Response for QPGS0Response {
    fn decode(src: &mut BytesMut) -> Result<Self> {
        debug!("Input: {:?}", from_utf8(&src)?);

        Ok(Self {
           from_utf8(&src)
        })
    }
}