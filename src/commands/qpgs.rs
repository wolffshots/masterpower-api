use crate::command::{Command, Response};
use crate::error::{Result};
use bytes::BytesMut;
use serde_derive::Serialize;
use std::str::from_utf8;
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
}

impl Response for QGPSResponse {
    fn decode(src: &mut BytesMut) -> Result<Self> {
        debug!("Input: {:?}", from_utf8(&src)?);
        Ok(Self {})
    }
}