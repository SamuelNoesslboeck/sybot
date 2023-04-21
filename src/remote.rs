use alloc::sync::Arc;
use std::sync::Mutex;

use stepper_lib::units::*;

/// A `PushRemote` defines a remote connection that the robot can push values to
pub trait PushRemote<const C : usize> {
    /// Publish a set of phis to the remote connection
    fn publ_phis(&mut self, phis : &[Phi; C]) -> Result<(), crate::Error>;

    // fn pub_drive(&mut self);
}

// pub trait PullRemote<const C : usize> : PushRemote<C> {
//     fn write_link<const DECO : usize, const DIM : usize, const ROT : usize>(&mut self, robot : dyn Robot<C, DECO, DIM, ROT>);

//     fn link<R, const DECO : usize, const DIM : usize, const ROT : usize>(&mut self) -> Option<&Rc<RefCell<R>>> 
//         where 
//             R : Robot<C, DECO, DIM, ROT>;
    
//     fn link_mut<R, const DECO : usize, const DIM : usize, const ROT : usize>(&mut self) -> Option<&mut Rc<RefCell<R>>> 
//         where 
//             R : Robot<C, DECO, DIM, ROT>;
// }   

impl<T : PushRemote<C>, const C : usize> PushRemote<C> for Arc<Mutex<T>> {
    fn publ_phis(&mut self, phis : &[Phi; C]) -> Result<(), crate::Error> {
        let mut rem = self.lock().unwrap(); 
        rem.publ_phis(phis)
    }

    // fn pub_drive(&mut self) {
    //     // let mut rem = self.lock().unwrap(); 
    //     // rem.pub_drive(phis)
        
    //     todo!();
    // }
}