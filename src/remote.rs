use stepper_lib::units::*;
use sybot_robs::PushRemote;

// Submodules 
    #[cfg(feature = "http")]
    pub mod http;

    #[cfg(feature = "mqtt")]
    pub mod mqtt;
// 

pub struct ConsoleRemote { 
    pub phis : Vec<Phi>
} 

impl PushRemote for ConsoleRemote {
    fn push_phis(&mut self, phis : &[stepper_lib::units::Phi]) -> Result<(), sybot_robs::Error> {
        self.phis = phis.to_vec();
        Ok(())
    }
    
    fn push_other(&mut self, other : sybot_robs::PushMsg) -> Result<(), sybot_robs::Error> {
        Ok(())
    }

    fn push_any(&mut self, msg_type : &str, msg : &[u8]) -> Result<(), sybot_robs::Error> {
        Ok(())
    }
}

pub struct FileRemote { 
    pub path: String
}