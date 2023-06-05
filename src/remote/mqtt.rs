use core::mem::size_of;
use core::time::Duration;

use paho_mqtt::{Client, ServerResponse, MessageBuilder};
use stepper_lib::units::*;

use crate::PushRemote;

// Topics
const TOPIC_PHIS : &str = "pos/phis";

// Helpers
fn phis_to_bytes(phis : &[Phi]) -> Vec<u8> {
    let mut bytes = vec![0u8; phis.len() * 4];
    
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
    pub fn new(host : &str) -> Result<Self, crate::Error> {
        let mut client = Client::new(host)?;
        client.set_timeout(Duration::from_secs(5));
        
        Ok(Self {
            client
        })
    }

    pub fn connect(&self) -> Result<ServerResponse, crate::Error> {
        Ok(self.client.connect(None)?)
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

    fn push_other(&mut self, other : sybot_robs::PushMsg) -> Result<(), sybot_robs::Error> {
        Ok(())
    }

    fn push_any(&mut self, msg_type : &str, msg : &[u8]) -> Result<(), sybot_robs::Error> {
        Ok(())
    }
}