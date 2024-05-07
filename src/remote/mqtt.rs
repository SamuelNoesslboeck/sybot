use core::mem::size_of;
use core::time::Duration;

use paho_mqtt::{Client, ServerResponse, MessageBuilder, CreateOptionsBuilder};
use syunit::*;

use crate::PushRemote;

// Topics
const TOPIC_PHIS : &str = "pos/phis";

// Helpers
fn phis_to_bytes(phis : &[Phi]) -> Vec<u8> {
    let mut bytes = vec![0u8; phis.len() * size_of::<f32>()];
    
    for i in 0 .. phis.len() {
        let p_bytes = phis[i].0.to_le_bytes();

        for n in 0 .. size_of::<f32>() {
            bytes[i*size_of::<f32>() + n] = p_bytes[n];
        }
    }

    bytes
}

pub struct Publisher {
    client : Client
}

impl Publisher {
    pub fn new<H : Into<String>, S : Into<String>>(host : H, id : S) -> Result<Self, crate::Error> {
        let builder = CreateOptionsBuilder::new()
            .client_id(id)
            .server_uri(host);

        let mut client = Client::new(builder.finalize())?;
        
        client.set_timeout(Duration::from_secs(5));
        
        Ok(Self {
            client
        })
    }

    pub fn connect(&self) -> Result<ServerResponse, crate::Error> {
        Ok(self.client.connect(None)?)
    }

    pub fn is_connected(&self) -> bool {
        self.client.is_connected()
    }
}

impl PushRemote for Publisher {
    fn push_phis(&mut self, phis : &[Phi]) -> Result<(), crate::Error> {
        let msg = MessageBuilder::new()
            .topic(TOPIC_PHIS)
            .payload(phis_to_bytes(phis))
            .qos(0)
            .finalize();

        Ok(self.client.publish(msg)?)
    }

    fn push_other(&mut self, _other : crate::remote::PushMsg) -> Result<(), crate::Error> {
        Ok(())
    }

    fn push_any(&mut self, _msg_type : &str, _msg : &[u8]) -> Result<(), crate::Error> {
        Ok(())
    }
}