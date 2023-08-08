// use syact::units::*;

// Submodules 
    #[cfg(feature = "http")]
    pub mod http;

    #[cfg(feature = "mqtt")]
    pub mod mqtt;

    #[cfg(feature = "msg")]
    pub mod msg;
// 

// pub struct ConsoleRemote { 
//     pub phis : Vec<Phi>
// } 

// impl PushRemote for ConsoleRemote {
//     fn push_phis(&mut self, phis : &[syact::units::Phi]) -> Result<(), crate::robs::Error> {
//         self.phis = phis.to_vec();
//         Ok(())
//     }
    
//     fn push_other(&mut self, other : crate::robs::PushMsg) -> Result<(), crate::robs::Error> {
//         Ok(())
//     }

//     fn push_any(&mut self, msg_type : &str, msg : &[u8]) -> Result<(), crate::robs::Error> {
//         Ok(())
//     }
// }

// pub struct FileRemote { 
//     pub path: String
// }