use core::future::Future;
use core::task::Poll;

use syact::{Tool, SyncComp, Setup};
use syact::ctrl::pin::{UniPin, UniInPin};

pub trait Device : Setup {
    fn tool(&self) -> Option<&dyn Tool> { 
        None 
    }

    fn comp(&self) -> Option<&dyn SyncComp> {
        None
    }
}

pub struct PinWait<'a> { 
    pin : &'a UniInPin,
    sig : bool
}

impl<'a> Future for PinWait<'a> {
    type Output = Result<(), crate::Error>;

    fn poll(self : std::pin::Pin<&mut Self>, _ : &mut std::task::Context<'_>) -> std::task::Poll<Self::Output> {
        let this = self.get_mut();
    
        if this.pin.is_high() == this.sig {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

pub struct Button {
    pin : UniInPin
}

impl Button {
    pub fn new(pin : u8) -> Result<Self, crate::Error> {
        Ok(Self { 
            pin: UniPin::new(pin)?.into_input()
        })
    }

    pub fn wait(&self, sig : bool) -> PinWait {
        PinWait { 
            pin: &self.pin, 
            sig
        }
    }
}

// pub struct CameraDevice {

// }