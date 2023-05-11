use sybot_pkg::{RobotInfo, CompInfo, Package};
use sybot_rcs::WorldObj;

use crate::{InfoRobot, Vars, BasicRobot};

#[derive(Debug)]
pub struct TheoRobot {
    info : RobotInfo,
    vars : Vars
}

impl TryFrom<Package> for TheoRobot {
    fn try_from(pkg: Package) -> Result<Self, Self::Error> {
        Ok(Self {

        })
    }
}

impl InfoRobot for TheoRobot {
    fn info<'a>(&'a self) -> &'a RobotInfo {
        &self.info
    }

    fn vars<'a>(&'a self) -> &'a Vars {
        &self.vars
    }

    fn vars_mut<'a>(&'a mut self) -> &'a mut Vars {
        &mut self.vars
    }
}

#[derive(Debug)]
pub struct SimpleRobot<const C : usize> {
    info : RobotInfo,
    vars : Vars,

    cinfos : [CompInfo; C], 
    comps : [Box<dyn stepper_lib::SyncComp>; C]
}

impl<const C : usize> InfoRobot for SimpleRobot<C> {
    fn info<'a>(&'a self) -> &'a RobotInfo {
        &self.info
    }

    fn vars<'a>(&'a self) -> &'a Vars {
        &self.vars
    }

    fn vars_mut<'a>(&'a mut self) -> &'a mut Vars {
        &mut self.vars
    }
}

impl<const C : usize> BasicRobot<C> for SimpleRobot<C> {
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