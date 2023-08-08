// use syact::units::*;

// Submodules 
    #[cfg(feature = "http")]
    pub mod http;

    #[cfg(feature = "mqtt")]
    pub mod mqtt;

    #[cfg(feature = "msg")]
    pub mod msg;
// 

pub enum PushMsg {
    Measurement,
    ToolChange
}

/// A `PushRemote` defines a remote connection that the robot can push values to
pub trait PushRemote {
    /// Publish a set of phis to the remote connection
    fn push_phis(&mut self, phis : &[Phi]) -> Result<(), crate::Error>;

    fn push_other(&mut self, other : PushMsg) -> Result<(), crate::Error>;

    fn push_any(&mut self, msg_type : &str, msg : &[u8]) -> Result<(), crate::Error>;
}
