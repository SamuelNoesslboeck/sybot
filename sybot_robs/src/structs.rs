use stepper_lib::{Setup, Tool};
use stepper_lib::units::*;
use sybot_pkg::{RobotInfo, CompInfo, Package};
use sybot_rcs::WorldObj;

use crate::{InfoRobot, Vars, BasicRobot, PushRemote, RobotDesc, ComplexRobot, SegmentChain};

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

impl<const C : usize> TryFrom<Package> for BasicStepperRobot<C> {
    fn try_from(pkg: Package) -> Result<Self, Self::Error> {
        
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
        Ok(())
    }

    fn valid_phis(&self, phis : [Phi; C]) -> Result<(), crate::Error> {
        let gammas = self.gammas_from_phis(phis);
        if self.comps().valid_gammas(&gammas) {
            Ok(())
        } else {
            Err(format!("The given phis are not valid! {:?}", self.comps().valid_gammas_verb(&gammas)).into())
        }
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
    // Info 
        info : RobotInfo,
        vars : Vars<C>,
    // 

    // Basic
        cinfos : [CompInfo; C], 
        comps : [Box<dyn stepper_lib::SyncComp>; C],

        tools : Vec<Box<dyn Tool + std::marker::Send>>,
        tool_id : Option<usize>,

        remotes : Vec<Box<dyn PushRemote>>,
    //

    // Complex
        wobj : WorldObj,
        desc : Option<Box<dyn RobotDesc<C>>>,
        seg_chain : Box<dyn SegmentChain<C>>
    // 
}

impl<const C : usize> TryFrom<Package> for ComplexStepperRobot<C> {
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

impl<const C : usize> Setup for ComplexStepperRobot<C> {
    fn setup(&mut self) -> Result<(), stepper_lib::Error> {
        self.comps_mut().setup()
    }
}

impl<const C : usize> InfoRobot<C> for ComplexStepperRobot<C> {
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

impl<const C : usize> BasicRobot<C> for ComplexStepperRobot<C> {
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

    fn valid_phis(&self, phis : [Phi; C]) -> Result<(), crate::Error> {
        let gammas = self.gammas_from_phis(phis);
        if self.comps().valid_gammas(&gammas) {
            Ok(())
        } else {
            Err(format!("The given phis are not valid! {:?}", self.comps().valid_gammas_verb(&gammas)).into())
        }
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

impl<const C : usize> ComplexRobot<C> for ComplexStepperRobot<C> {
    // RCS
        fn wobj<'a>(&'a self) -> &'a WorldObj {
            &self.wobj
        }

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj {
            &mut self.wobj
        }
    // 

    // Movement
        fn move_l(&mut self, _ : [Delta; C]) -> Result<(), crate::Error> {
            todo!()
        }

        fn move_l_abs(&mut self, _ : [Gamma; C]) -> Result<(), crate::Error> {
            todo!()
        }
    //

    // Description
        fn set_desc(&mut self, desc : Box<dyn RobotDesc<C>>) {
            self.desc = Some(desc)
        }

        fn desc<'a>(&'a self) -> Option<&'a Box<dyn RobotDesc<C>>> {
            if let Some(desc) = &self.desc {
                Some(desc)
            } else {
                None
            }
        }

        fn desc_mut<'a>(&'a mut self) -> Option<&'a mut Box<dyn RobotDesc<C>>> {
            if let Some(desc) = &mut self.desc {
                Some(desc) 
            } else {
                None
            }
        }
    // 

    // Segments
        fn seg_chain<'a>(&'a self) -> &'a Box<dyn crate::SegmentChain<C>> {
            &self.seg_chain
        }

        fn seg_chain_mut<'a>(&'a mut self) -> &'a mut Box<dyn SegmentChain<C>> {
            &mut self.seg_chain
        }
    // 
}