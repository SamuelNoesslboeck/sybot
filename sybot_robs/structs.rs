use core::ops::DerefMut;

use stepper_lib::{Setup, Tool};
use stepper_lib::meas::SimpleMeas;
use stepper_lib::units::*;
use sybot_pkg::{RobotInfo, Package, AngConf};
use sybot_rcs::Position;

use crate::{InfoRobot, Vars, BasicRobot, PushRemote, RobotDesc, Device, DeviceManager};

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
pub struct StepperRobot<const C : usize> {
    // Basic
    info : RobotInfo,
    vars : Vars<C>,

    ang_confs : Vec<AngConf>,
    comps : [Box<dyn stepper_lib::SyncComp>; C],
    meas : Vec<Box<dyn SimpleMeas>>, 

    devices : Vec<Device>,
    tools : Vec<Box<dyn Tool>>,
    tool_id : Option<usize>,

    remotes : Vec<Box<dyn PushRemote>>
}

impl<const C : usize> StepperRobot<C> {
    pub fn new(info : RobotInfo, ang_confs : Vec<AngConf>, comps : [Box<dyn stepper_lib::SyncComp>; C], 
    meas: Vec<Box<dyn SimpleMeas>>, tools : Vec<Box<dyn Tool>>) -> Self {
        Self {
            info,
            vars: Vars::default(),

            ang_confs,
            comps,
            meas,
            
            devices: Vec::new(),
            tools,
            tool_id: None,

            remotes: Vec::new()
        }
    }
}

impl<const C : usize> TryFrom<&Package> for StepperRobot<C> {
    type Error = crate::Error;
    
    fn try_from(pkg: &Package) -> Result<Self, Self::Error> {
        let comps = pkg.parse_components()?;
        let tools = pkg.parse_tools()?;
        let meas = pkg.parse_meas()?;
        let ang_confs = pkg.parse_ang_confs().unwrap();

        let mut rob = Self::new(
            pkg.info.clone(), ang_confs, comps, meas, tools
        );

        if let Some(devs) = &pkg.devices {
            for d in devs {
                rob.devices.push(
                    Device::new(d.name.clone(), pkg.libs.parse_tool(d).unwrap())
                );
            }
        }

        if let Some(link) = pkg.lk.clone() {
            rob.comps_mut().write_link(link);
        }

        Ok(rob)
    }
}

impl<const C : usize> Setup for StepperRobot<C> {
    fn setup(&mut self) -> Result<(), stepper_lib::Error> {
        self.comps_mut().setup()?;

        for meas in &mut self.meas {
            meas.setup()?;
        }

        for device in &mut self.devices {
            device.mount();
        }

        Ok(())
    }
}

impl<const C : usize> InfoRobot<C> for StepperRobot<C> {
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

impl<const C : usize> BasicRobot<C> for StepperRobot<C> {
    // Data
        fn ang_confs<'a>(&'a self) -> &'a [sybot_pkg::AngConf] {
            &self.ang_confs
        }

        fn comps<'a>(&'a self) -> &'a dyn stepper_lib::SyncCompGroup<dyn stepper_lib::SyncComp, C> {
            &self.comps
        }

        fn comps_mut<'a>(&'a mut self) -> &'a mut dyn stepper_lib::SyncCompGroup<dyn stepper_lib::SyncComp, C> {
            &mut self.comps
        }
    //

    // Movement
        fn move_p_sync(&mut self, desc : &mut dyn RobotDesc<C>, p : Position, speed_f : f32) -> Result<[Delta; C], crate::Error> {
            let phis = desc.convert_pos(self, p)?;
            let gammas = self.gammas_from_phis(phis);
            self.move_abs_j_sync(
                gammas,
                speed_f
            )
        }
    //

    fn update(&mut self) -> Result<(), crate::Error> {
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

    // Device
        fn device_manager(&self) -> Option<&dyn DeviceManager> {
            Some(self)
        }

        fn device_manager_mut(&mut self) -> Option<&mut dyn DeviceManager> {
            Some(self)
        }
    // 

    // Meas
        fn full_meas(&mut self) -> Result<(), crate::Error> {
            for i in 0 .. C {
                self.meas[i].measure(self.comps[i].deref_mut())?;
            }

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

impl<const C : usize> DeviceManager for StepperRobot<C> {
    fn add_device(&mut self, device: Device) {
        self.devices.push(device);
    }

    fn remove_device(&mut self, index : usize) -> Device {
        self.devices.remove(index)
    }

    fn get_device(&self, index : usize) -> Option<&Device> {
        self.devices.get(index)
    }
    
    fn get_device_mut(&mut self, index : usize) -> Option<&mut Device> {
        self.devices.get_mut(index)
    }

    fn get_device_name(&self, name: &str) -> Option<&Device> {
        self.devices.iter().find(|d| d.name == name)
    }

    fn get_device_name_mut(&mut self, name: &str) -> Option<&mut Device> {
        self.devices.iter_mut().find(|d| d.name == name)
    }
}

// pub struct ComplexCollective<const C : usize> {

// }

// pub struct ComplexStepperRobot<const C : usize> {
//     // Info 
//         info : RobotInfo,
//         vars : Vars<C>,
//     // 

//     // Basic
//         ang_confs : Vec<AngConf>,
//         comps : [Box<dyn stepper_lib::SyncComp>; C],

//         tools : Vec<Box<dyn Tool>>,
//         tool_id : Option<usize>,

//         remotes : Vec<Box<dyn PushRemote>>,
//     //
// }

// impl<const C : usize> ComplexStepperRobot<C> {
//     pub fn new(info : RobotInfo, ang_confs : Vec<AngConf>, comps : [Box<dyn stepper_lib::SyncComp>; C], 
//     tools : Vec<Box<dyn Tool>>, wobj : WorldObj) -> Self {
//         Self {
//             info,
//             vars: Vars::default(),

//             ang_confs,
//             comps,
            
//             tools,
//             tool_id: None,

//             remotes: vec![],

//             wobj
//         }
//     }
// }

// impl<const C : usize> ComplexStepperRobot<C> {
//     fn try_from_pkg(pkg: Package, ) -> Result<Self, crate::Error> {
//         let comps = pkg.parse_components()?;
//         let tools = pkg.parse_tools()?;
//         let ang_confs = pkg.parse_ang_confs().unwrap();

//         let wobj = if let Some(wobj) = pkg.wobj {
//             wobj
//         } else {
//             return Err("No coordinate system has been defined in the package! (rcs/init.json)".into());
//         };
    
//         let mut rob = Self::new(
//             pkg.info, ang_confs, comps, tools, wobj
//         );

//         if let Some(link) = pkg.lk {
//             rob.comps_mut().write_link(link);
//         }

//         Ok(rob)
//     }
// }

// impl<const C : usize> Setup for ComplexStepperRobot<C> {
//     fn setup(&mut self) -> Result<(), stepper_lib::Error> {
//         self.comps_mut().setup()
//     }
// }

// impl<const C : usize> InfoRobot<C> for ComplexStepperRobot<C> {
//     fn info<'a>(&'a self) -> &'a RobotInfo {
//         &self.info
//     }

//     fn vars<'a>(&'a self) -> &'a Vars<C> {
//         &self.vars
//     }

//     fn vars_mut<'a>(&'a mut self) -> &'a mut Vars<C> {
//         &mut self.vars
//     }
// }

// impl<const C : usize> BasicRobot<C> for ComplexStepperRobot<C> {
//     // Data
//         fn ang_confs<'a>(&'a self) -> &'a [AngConf] {
//             &self.ang_confs
//         }

//         fn comps<'a>(&'a self) -> &'a dyn stepper_lib::SyncCompGroup<dyn stepper_lib::SyncComp, C> {
//             &self.comps
//         }

//         fn comps_mut<'a>(&'a mut self) -> &'a mut dyn stepper_lib::SyncCompGroup<dyn stepper_lib::SyncComp, C> {
//             &mut self.comps
//         }
//     // 

//     fn update(&mut self) -> Result<(), crate::Error> {
//         todo!()
//     }

//     fn valid_phis(&self, phis : [Phi; C]) -> Result<(), crate::Error> {
//         let gammas = self.gammas_from_phis(phis);
//         if self.comps().valid_gammas(&gammas) {
//             Ok(())
//         } else {
//             Err(format!("The given phis are not valid! {:?}", self.comps().valid_gammas_verb(&gammas)).into())
//         }
//     }

//     // Tools
//         fn get_tool(&self) -> Option<&Box<dyn stepper_lib::Tool + std::marker::Send>> {
//             if let Some(tool_id) = self.tool_id {
//                 self.tools.get(tool_id)
//             } else {
//                 None 
//             }
//         }

//         fn get_tool_mut(&mut self) -> Option<&mut Box<dyn stepper_lib::Tool + std::marker::Send>> {
//             if let Some(tool_id) = self.tool_id {
//                 self.tools.get_mut(tool_id)
//             } else {
//                 None 
//             }
//         }

//         fn get_tools(&self) -> &Vec<Box<dyn stepper_lib::Tool + std::marker::Send>> {
//             &self.tools 
//         }

//         fn set_tool_id(&mut self, tool_id : Option<usize>) -> Option<&mut Box<dyn stepper_lib::Tool + std::marker::Send>> {
//             if let Some(id) = tool_id {   
//                 if id < self.tools.len() {
//                     self.tool_id = tool_id;
//                     Some(&mut self.tools[id])
//                 } else {
//                     None
//                 }
//             } else {
//                 None
//             }
//         }
//     // 

//     // Remote
//         fn add_remote(&mut self, remote : Box<dyn crate::PushRemote>) {
//             self.remotes.push(remote)
//         }

//         fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn crate::PushRemote>> {
//             &self.remotes
//         }

//         fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn crate::PushRemote>> {
//             &mut self.remotes
//         }
//     //
// }

// impl<const C : usize> ComplexRobot<C> for StepperRobot<C> {
//     // Movement
//         fn move_l(&mut self, desc : &mut dyn RobotDesc<C>, _ : [Delta; C]) -> Result<(), crate::Error> {
//             todo!()
//         }

//         fn move_abs_l(&mut self, desc : &mut dyn RobotDesc<C>, _ : [Gamma; C]) -> Result<(), crate::Error> {
//             todo!()
//         }

//         fn move_j(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<(), crate::Error> {
//             todo!()
//         }

//         fn move_abs_j(&mut self, gammas : [Gamma; C], speed_f : f32) -> Result<(), crate::Error> {
//             todo!()
//         }
//     //
// }