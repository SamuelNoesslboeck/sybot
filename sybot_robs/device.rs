use core::ops::{Deref, DerefMut};

use stepper_lib::Tool;

pub struct Device {
    pub name: String,
    pub dev : Box<dyn Tool> 
}

impl Device {
    pub fn new(name : String, dev : Box<dyn Tool>) -> Self {
        Self {
            name,
            dev
        }
    }
}

impl Deref for Device {
    type Target = Box<dyn Tool>;

    fn deref(&self) -> &Self::Target {
        &self.dev
    }
}

impl DerefMut for Device {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.dev
    }
}

pub trait DeviceManager {
    fn add_device(&mut self, device: Device);
    fn remove_device(&mut self, index : usize) -> Device;

    fn get_device(&self, index : usize) -> Option<&Device>;
    fn get_device_mut(&mut self, index : usize) -> Option<&mut Device>;

    fn get_device_name(&self, name: &str) -> Option<&Device>;
    fn get_device_name_mut(&mut self, name: &str) -> Option<&mut Device>;
}