use core::ops::DerefMut;

use glam::Vec3;
use stepper_lib::{Setup, Tool, SyncCompGroup, SyncComp};
use stepper_lib::comp::stepper::{StepperComp, StepperCompGroup};
use stepper_lib::meas::SimpleMeas;
use stepper_lib::units::*;
use sybot_pkg::{RobotInfo, Package, AngConf};
use sybot_rcs::Position;

use crate::{InfoRobot, Vars, BasicRobot, PushRemote, Descriptor, ComplexRobot};

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

// #[derive(Debug)]
pub struct StepperRobot<T : SyncCompGroup<C>, const C : usize> {
    // Basic
    info : RobotInfo,
    vars : Vars<C>,

    ang_confs : Vec<AngConf>,
    comps : T,
    meas : Vec<Box<dyn SimpleMeas>>, 

    tools : Vec<Box<dyn Tool>>,
    tool_id : Option<usize>,

    remotes : Vec<Box<dyn PushRemote>>
}

impl<T : SyncCompGroup<C>, const C : usize> StepperRobot<T, C> {
    pub fn new(info : RobotInfo, ang_confs : Vec<AngConf>, comps : T, 
    meas: Vec<Box<dyn SimpleMeas>>, tools : Vec<Box<dyn Tool>>) -> Self {
        Self {
            info,
            vars: Vars::default(),

            ang_confs,
            comps,
            meas,
            
            tools,
            tool_id: None,

            remotes: Vec::new()
        }
    }

    pub fn from_pkg_stat(pkg : &Package, comps : T) -> Result<Self, crate::Error> {
        let tools = pkg.parse_tools_dyn()?;
        let meas = pkg.parse_meas_dyn()?;
        let ang_confs = pkg.parse_ang_confs().unwrap();

        let mut rob = Self::new(
            pkg.info.clone(), ang_confs, comps, meas, tools
        );

        // if let Some(devs) = &pkg.devices {
        //     for d in devs {
        //         rob.devices.push(
        //             Device::new(d.name.clone(), pkg.libs.parse_tool_dyn(d).unwrap())
        //         );
        //     }
        // }    TODO: Add dynamic devices

        if let Some(link) = &pkg.lk {
            rob.comps_mut().write_link(link.clone());
        }

        Ok(rob)
    }
}

impl<const C : usize> TryFrom<&Package> for StepperRobot<[Box<dyn SyncComp>; C], C> {
    type Error = crate::Error;
    
    fn try_from(pkg: &Package) -> Result<Self, Self::Error> {
        let comps = pkg.parse_comps_dyn()?;
        Self::from_pkg_stat(pkg, comps)
    }
}

impl<T : SyncCompGroup<C>, const C : usize> Setup for StepperRobot<T, C> {
    fn setup(&mut self) -> Result<(), stepper_lib::Error> {
        self.comps_mut().setup()?;

        for meas in &mut self.meas {
            meas.setup()?;
        }

        Ok(())
    }
}

impl<T : SyncCompGroup<C>, const C : usize> InfoRobot<C> for StepperRobot<T, C> {
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

impl<T : SyncCompGroup<C>, const C : usize> BasicRobot<C> for StepperRobot<T, C> {
    // Data
        fn ang_confs<'a>(&'a self) -> &'a [sybot_pkg::AngConf] {
            &self.ang_confs
        }

        fn comps<'a>(&'a self) -> &'a dyn stepper_lib::SyncCompGroup<C> {
            &self.comps
        }

        fn comps_mut<'a>(&'a mut self) -> &'a mut dyn stepper_lib::SyncCompGroup<C> {
            &mut self.comps
        }
    //

    // Movement
        fn move_p_sync(&mut self, desc : &mut dyn Descriptor<C>, p : Position, speed_f : f32) -> Result<[Delta; C], crate::Error> {
            let phis = desc.convert_pos(self, p)?;
            self.move_abs_j_sync(
                phis,
                speed_f
            )
        }
    //

    fn update(&mut self) -> Result<(), crate::Error> {
        let phis = self.phis();
        for rem in &mut self.remotes {
            rem.push_phis(&phis)?;
        }

        Ok(())
    }

    // Tools
        fn get_tool(&self) -> Option<&Box<dyn Tool>> {
            if let Some(tool_id) = self.tool_id {
                self.tools.get(tool_id)
            } else {
                None 
            }
        }

        fn get_tool_mut(&mut self) -> Option<&mut Box<dyn Tool>> {
            if let Some(tool_id) = self.tool_id {
                self.tools.get_mut(tool_id)
            } else {
                None 
            }
        }

        fn get_tools(&self) -> &Vec<Box<dyn Tool>> {
            &self.tools 
        }

        fn set_tool_id(&mut self, tool_id : Option<usize>) -> Option<&mut Box<dyn Tool>> {
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

    // Meas
        fn move_home(&mut self) -> Result<(), crate::Error> {
            for i in 0 .. C {
                self.meas[i].measure(self.comps.index_mut(i).deref_mut())?;
            }

            self.update()?;

            Ok(())
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

    //
}

impl<T : StepperCompGroup<C>, const C : usize> ComplexRobot<C> for StepperRobot<T, C> {
    fn move_l(&mut self, desc : &mut dyn Descriptor<C>, distance : Vec3, accuracy : f32) -> Result<(), crate::Error> {
        // let pos_0 = desc.current_tcp().pos();

        // let poses = sybot_rcs::math::split_linear(pos_0, distance, accuracy);
        // let mut builder = self.comps.create_path_builder([Omega::ZERO; C]);
        
        // let mut tstack = Vec::new();
        // let mut dstack = Vec::new();

        // for pos in poses {
        //     dstack.push(
        //         desc.convert_pos(self, pos)?
        //     )
        // }

        // builder.generate(&mut tstack, &mut dstack);

        Ok(())
    }

    fn move_abs_l(&mut self, desc : &mut dyn Descriptor<C>, pos : Vec3, accuracy : f32) -> Result<(), crate::Error> {
        Ok(())
    }
}