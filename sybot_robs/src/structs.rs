use stepper_lib::Setup;
use sybot_pkg::{RobotInfo, CompInfo, Package};

use crate::{InfoRobot, Vars, BasicRobot};

#[derive(Debug)]
pub struct TheoRobot<const C : usize> {
    info : RobotInfo,
    vars : Vars<C>
}

impl<const C : usize> TryFrom<Package> for TheoRobot<C> {
    type Error = crate::Error;

    fn try_from(pkg: Package) -> Result<Self, Self::Error> {
        Ok(Self {
            info: pkg.info,
            vars: Vars::default()
        })
    }
}

impl<const C : usize> InfoRobot<C> for TheoRobot<C> {
    fn info<'a>(&'a self) -> &'a RobotInfo {
        &self.info
    }

    fn vars<'a>(&'a self) -> &'a Vars<C> {
        &self.vars
    }

    fn vars_mut<'a>(&'a mut self) -> &'a mut Vars<C> {
        &mut self.vars
    }
}

#[derive(Debug)]
pub struct BasicStepperRobot<const C : usize> {
    info : RobotInfo,
    vars : Vars<C>,

    cinfos : [CompInfo; C], 
    comps : [Box<dyn stepper_lib::SyncComp>; C]
}

impl<const C : usize> TryFrom<Package> for BasicStepperRobot<C> {
    type Error = crate::Error;

    fn try_from(pkg: Package) -> Result<Self, Self::Error> {
        if let Some(cinfos) = pkg.comps {
            if cinfos.len() != C {
                return Err(format!("Robot requires {} components, but {} were given!", C, cinfos.len()).into());
            }

            
        }

        Err("No components declared!".into())
    }
}

impl<const C : usize> Setup for BasicStepperRobot<C> {
    fn setup(&mut self) -> Result<(), stepper_lib::Error> {
        self.comps_mut().setup()
    }
}

impl<const C : usize> InfoRobot<C> for BasicStepperRobot<C> {
    fn info<'a>(&'a self) -> &'a RobotInfo {
        &self.info
    }

    fn vars<'a>(&'a self) -> &'a Vars<C> {
        &self.vars
    }

    fn vars_mut<'a>(&'a mut self) -> &'a mut Vars<C> {
        &mut self.vars
    }
}

impl<const C : usize> BasicRobot<C> for BasicStepperRobot<C> {
    fn cinfos<'a>(&'a self) -> &'a [sybot_pkg::CompInfo] {
        &self.cinfos
    }

    fn comps<'a>(&'a self) -> &'a dyn stepper_lib::SyncCompGroup<dyn stepper_lib::SyncComp, C> {
        &self.comps
    }

    fn comps_mut<'a>(&'a mut self) -> &'a mut dyn stepper_lib::SyncCompGroup<dyn stepper_lib::SyncComp, C> {
        &mut self.comps
    }

    fn update(&mut self) -> Result<(), crate::Error> {
        todo!()
    }
}

pub struct ComplexStepperRobot<const C : usize> {
    
}