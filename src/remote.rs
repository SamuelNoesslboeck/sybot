use stepper_lib::units::*;

pub trait Remote<const COMP : usize> {
    fn pub_phis(&mut self, phis : &[Phi; COMP]) -> Result<(), crate::Error>;

    fn pub_drive(&mut self);
}