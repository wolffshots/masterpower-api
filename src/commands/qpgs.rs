use crate::command::{Command, Response};
use crate::error::{Result};
use bytes::BytesMut;
use bytes::BufMut;
use serde_derive::Serialize;
use std::str::from_utf8;
use log::{debug};

pub struct QPGS;

impl Command for QPGS {
    const PROTOCOL_ID: &'static [u8] = b"QPGS";
    const COMMAND_NAME: &'static str = "QuerySpecificDeviceGeneralStatus";

    type Request = ();
    type Response = QGPSResponse;
}

pub trait Request {
    fn encode(&self) -> Result<Option<BytesMut>> {
        let mut buf = BytesMut::with_capacity(64);
        buf.put_u8(b'h');
        buf.put_u8(b'e');
        buf.put(&b"llo"[..]);
        Ok(Some(buf))
    }
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