use alloc::rc::Rc;
use alloc::sync::Arc;
use core::cell::RefCell;

use std::sync::Mutex;

use stepper_lib::units::*;

use crate::ConfRobot;

pub trait PushRemote<const COMP : usize> {
    fn pub_phis(&mut self, phis : &[Phi; COMP]) -> Result<(), crate::Error>;

    fn pub_drive(&mut self);
}

pub trait PullRemote<const COMP : usize> : PushRemote<COMP> {
    fn write_link<const DECO : usize, const DIM : usize, const ROT : usize>(&mut self, robot : dyn ConfRobot<COMP, DECO, DIM, ROT>);

    fn link<R, const DECO : usize, const DIM : usize, const ROT : usize>(&mut self) -> Option<&Rc<RefCell<R>>> 
        where 
            R : ConfRobot<COMP, DECO, DIM, ROT>;
    
    fn link_mut<R, const DECO : usize, const DIM : usize, const ROT : usize>(&mut self) -> Option<&mut Rc<RefCell<R>>> 
        where 
            R : ConfRobot<COMP, DECO, DIM, ROT>;
}   

impl<T : PushRemote<COMP>, const COMP : usize> PushRemote<COMP> for Arc<Mutex<T>> {
    fn pub_phis(&mut self, phis : &[Phi; COMP]) -> Result<(), crate::Error> {
        let mut rem = self.lock().unwrap(); 
        rem.pub_phis(phis)
    }

    fn pub_drive(&mut self) {
        // let mut rem = self.lock().unwrap(); 
        // rem.pub_drive(phis)

        todo!();
    }
}