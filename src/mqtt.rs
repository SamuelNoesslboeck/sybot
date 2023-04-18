use core::mem::size_of;
use core::time::Duration;

use paho_mqtt::{Client, ServerResponse, MessageBuilder};
use stepper_lib::units::*;

use crate::remote::PushRemote;

// Topics
const TOPIC_PHIS : &str = "pos/phis";

// Helpers
fn phis_to_bytes<const DIM : usize>(phis : &[Phi; DIM]) -> Vec<u8> {
    let mut bytes = vec![0u8; DIM * 4];
    
    for i in 0 .. DIM {
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
        let mut client = match Client::new(host) {
            Ok(c) => c, 
            Err(err) => return Err(crate::Error::new(std::io::ErrorKind::Other, err))
        };
        client.set_timeout(Duration::from_secs(5));
        
        Ok(Self {
            client
        })
    }

    pub fn connect(&self) -> Result<ServerResponse, crate::Error> {
        match self.client.connect(None) {
            Ok(res) => Ok(res),
            Err(err) => Err(crate::Error::new(std::io::ErrorKind::Other, err))
        }
    }
}

impl<const DIM : usize> PushRemote<DIM> for Publisher {
    fn pub_phis(&mut self, phis : &[Phi; DIM]) -> Result<(), crate::Error> {
        let msg = MessageBuilder::new()
            .topic(TOPIC_PHIS)
            .payload(phis_to_bytes(phis))
            .qos(0)
            .finalize();

        match self.client.publish(msg) {
            Ok(res) => Ok(res),
            Err(err) => Err(crate::Error::new(std::io::ErrorKind::Other, err))
        }
    }
}