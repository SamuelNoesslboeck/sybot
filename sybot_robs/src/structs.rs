use stepper_lib::{Setup, SyncCompGroup, Tool};
use sybot_pkg::{RobotInfo, CompInfo, Package};

use crate::{InfoRobot, Vars, BasicRobot, PushRemote};

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
    comps : [Box<dyn stepper_lib::SyncComp>; C],

    tools : Vec<Box<dyn Tool + std::marker::Send>>,
    tool_id : Option<usize>,

    remotes : Vec<Box<dyn PushRemote>>
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

    fn valid_phis(&self, phis : [stepper_lib::units::Phi; C]) -> Result<(), crate::Error> {
        todo!()
    }

    // Tools
        fn get_tool(&self) -> Option<&Box<dyn stepper_lib::Tool + std::marker::Send>> {
            if let Some(tool_id) = self.tool_id {
                self.tools.get(tool_id)
            } else {
                None 
            }
        }

        fn get_tool_mut(&mut self) -> Option<&mut Box<dyn stepper_lib::Tool + std::marker::Send>> {
            if let Some(tool_id) = self.tool_id {
                self.tools.get_mut(tool_id)
            } else {
                None 
            }
        }

        fn get_tools(&self) -> &Vec<Box<dyn stepper_lib::Tool + std::marker::Send>> {
            &self.tools 
        }

        fn set_tool_id(&mut self, tool_id : Option<usize>) -> Option<&mut Box<dyn stepper_lib::Tool + std::marker::Send>> {
            if let Some(id) = tool_id {   
                if id < self.tools.len() {
                    self.tool_id = tool_id;
                    Some(&mut self.tools[id])
                } else {
                    None
                }
            } else {
                None
            }
        }
    // 

    // Remote
        fn add_remote(&mut self, remote : Box<dyn crate::PushRemote>) {
            self.remotes.push(remote)
        }

        fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn crate::PushRemote>> {
            &self.remotes
        }

        fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn crate::PushRemote>> {
            &mut self.remotes
        }
    //
}

pub struct ComplexStepperRobot<const C : usize> {
    
}

