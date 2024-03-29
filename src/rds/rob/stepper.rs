use glam::Vec3;
use serde::de::DeserializeOwned;
use syact::{Setup, Tool, SyncCompGroup};
use syact::comp::stepper::{StepperComp, StepperCompGroup};
use syact::meas::SimpleMeas;
use syact::units::*;

use crate::pkg::{RobotPackage, parse_struct};
use crate::pkg::info::AngConf;
use crate::rcs::Position;

use crate::remote::PushRemote;
use crate::{Vars, Robot, Descriptor, Mode, default_modes};

pub struct StepperRobot<T, const C : usize> 
where 
    T : StepperCompGroup<dyn StepperComp, C>
{
    _vars : Vars<C>,

    _ang_confs : Vec<AngConf>,
    _comps : T,
    meas : [Vec<Box<dyn SimpleMeas>>; C], 

    tools : Vec<Box<dyn Tool>>,
    tool_id : Option<usize>,

    modes : Vec<Mode>,
    mode_id : usize,

    remotes : Vec<Box<dyn PushRemote>>
}

impl<T : StepperCompGroup<dyn StepperComp, C>, const C : usize> StepperRobot<T, C>
where 
    T : StepperCompGroup<dyn StepperComp, C>
{
    pub fn new(ang_confs : Vec<AngConf>, comps : T, 
    meas: [Vec<Box<dyn SimpleMeas>>; C], tools : Vec<Box<dyn Tool>>) -> Self {
        Self {
            _vars: Vars::default(),

            _ang_confs: ang_confs,
            _comps: comps,
            meas,
            
            tools,
            tool_id: None,

            modes: Vec::from(default_modes()),
            mode_id: 0,

            remotes: Vec::new()
        }
    }
}

impl<T, const C : usize> TryFrom<RobotPackage> for StepperRobot<T, C> 
where 
    T: StepperCompGroup<dyn StepperComp, C>,
    T: DeserializeOwned
{
    type Error = crate::Error;

    #[allow(deprecated)]
    fn try_from(pkg : RobotPackage) -> Result<Self, Self::Error> {      // TODO: Remove this mess
        let tools = pkg.parse_tools_dyn()?;
        let meas = pkg.parse_meas_dyn()?;
        let ang_confs = pkg.parse_ang_confs().unwrap();
        let comps : T = parse_struct(pkg.comps.unwrap())?.0;

        let mut rob = Self::new(
            ang_confs, comps, meas.try_into().ok().unwrap(), tools
        );

        if let Some(link) = &pkg.data {
            <T as SyncCompGroup<dyn StepperComp, C>>::write_data(rob.comps_mut(), link.clone());
        }

        Ok(rob)
    }
}

impl<T : StepperCompGroup<dyn StepperComp, C>, const C : usize> Setup for StepperRobot<T, C> 
where 
    T : StepperCompGroup<dyn StepperComp, C>
{
    fn setup(&mut self) -> Result<(), syact::Error> {
        self.comps_mut().setup()?;

        for meas_vec in &mut self.meas {
            for meas in meas_vec {
                meas.setup()?;
            }
        }

        Ok(())
    }
}

impl<T, const C : usize> Robot<C> for StepperRobot<T, C> 
where 
    T : StepperCompGroup<dyn StepperComp, C>
{
    type Comp = dyn StepperComp;
    type CompGroup = T;    

    // Data
        #[inline]
        fn ang_confs<'a>(&'a self) -> &'a [AngConf] {
            &self._ang_confs
        }

        #[inline]
        fn comps<'a>(&'a self) -> &'a T {
            &self._comps
        }

        #[inline]
        fn comps_mut<'a>(&'a mut self) -> &'a mut T {
            &mut self._comps
        }
        
        #[inline]
        fn vars<'a>(&'a self) -> &'a Vars<C> {
            &self._vars
        }
        
        #[inline]
        fn vars_mut<'a>(&'a mut self) -> &'a mut Vars<C> {
            &mut self._vars
        }
    //

    // Movement
        fn move_p_sync<D : Descriptor<C>>(&mut self, desc : &mut D, p : Position, speed_f : f32) -> Result<[Delta; C], crate::Error> {
            let phis = desc.convert_pos(self, p)?;
            self.move_abs_j_sync(
                phis,
                speed_f
            )
        }
    //

    // Complex movement
        #[allow(unused)]
        fn move_j(&mut self, deltas : [Delta; C], gen_speed_f : f32) -> Result<(), crate::Error> {
            let speed_f = syact::math::movements::ptp_exact_unbuffered(self.comps_mut(), deltas, gen_speed_f);
            self.comps_mut().drive_rel_async(deltas, speed_f)
        }

        #[allow(unused)]
        fn move_abs_j(&mut self, gammas : [Gamma; C], gen_speed_f : f32) -> Result<(), crate::Error> {
            // TODO: Implement gammas to deltas function
            let mut deltas = [Delta::ZERO; C];
            let comp_gammas = self.comps().gammas();

            for i in 0 .. C {
                deltas[i] = gammas[i] - comp_gammas[i];
            }

            let speed_f = syact::math::movements::ptp_exact_unbuffered(self.comps_mut(), deltas, gen_speed_f);
            self.comps_mut().drive_rel_async(deltas, speed_f)
        }

        #[allow(unused)]
        fn move_l<D : Descriptor<C>>(&mut self, desc : &mut D, distance : Vec3, accuracy : f32, speed : Omega) -> Result<(), crate::Error> {
            todo!();

            // let pos_0 = desc.current_tcp().pos();

            // let poses = crate::rcs::math::split_linear(pos_0, distance, accuracy);
            
            // let mut gam_stack = Vec::new();
            // let mut dstack = Vec::new();

            // for pos in poses {
            //     let phis = desc.convert_pos(self, Position::new(pos))?;
            //     let mut gammas = self.gammas_from_phis(phis);

            //     // for i in 0 .. C {
            //     //     gammas[i] = self.comps.index(i).abs_super_gamma(gammas[i]);
            //     // }

            //     gam_stack.push(
            //         gammas
            //     )
            // }

            // for i in 1 .. gam_stack.len() {
            //     let mut deltas = [Delta::ZERO; C];

            //     for n in 0 .. C {
            //         deltas[n] = gam_stack[i][n] - gam_stack[i - 1][n];   
            //     }

            //     dstack.push(deltas);
            // }

            // let mut tstack = vec![accuracy / speed; dstack.len()];

            // let mut builder = self.comps.create_path_builder([Omega::ZERO; C]);
            // builder.generate(&mut tstack, dstack.as_slice(), [Some(Omega::ZERO); C]);

            // let mut nodes = builder.unpack();

            // println!("driving");

            // let mut corr = [(Delta::ZERO, Time::ZERO); C];
            // let mut t_err = Time::ZERO;

            // for i in 1 .. nodes.len() {
            //     self.comps.drive_node_to_node(&nodes[i - 1], &nodes[i], &mut corr)?;

            //     for n in 0 .. C {
            //         nodes[i][n].delta = nodes[i][n].delta + corr[n].0;
            //         t_err += corr[n].1;
            //     }
            // }

            // if let Some(last) = nodes.last() {
            //     self.comps.drive_nodes(&last, [Omega::ZERO; C], &mut corr)?;
            // }

            // dbg!(corr, t_err);

            // println!("Done");

            Ok(())
        }

        fn move_abs_l<D : Descriptor<C>>(&mut self, desc : &mut D, pos : Vec3, accuracy : f32, speed : Omega) -> Result<(), crate::Error> {
            let pos_0 = desc.tcp().pos();
            self.move_l(desc, pos - pos_0, accuracy, speed)
        }
    // 

    // Events
        fn update(&mut self) -> Result<(), crate::Error> {
            let phis = self.phis();
            for rem in &mut self.remotes {
                rem.push_phis(&phis)?;
            }

            Ok(())
        }
    // 

    // Tools
        fn get_tool(&self) -> Option<&dyn Tool> {
            if let Some(tool_id) = self.tool_id {
                self.tools.get(tool_id).and_then(|t| Some(t.as_ref()))
            } else {
                None 
            }
        }

        fn get_tool_mut(&mut self) -> Option<&mut dyn Tool> {
            if let Some(tool_id) = self.tool_id {
                if let Some(t) = self.tools.get_mut(tool_id) {
                    Some(t.as_mut()) 
                } else {
                    None
                }
            } else {
                None 
            }
        }

        fn get_tools(&self) -> &Vec<Box<dyn Tool>> {
            &self.tools 
        }

        fn set_tool_id(&mut self, tool_id : Option<usize>) -> Option<&mut dyn Tool> {
            if let Some(id) = tool_id {   
                if id < self.tools.len() {
                    self.tool_id = tool_id;
                    Some(self.tools[id].as_mut())
                } else {
                    None
                }
            } else {
                None
            }
        }
    // 

    // Mode
        fn mode(&self) -> &Mode {
            &self.modes[self.mode_id]
        }

        fn set_mode(&mut self, index : usize) -> Result<&Mode, crate::Error> {
            if index >= self.modes.len() {
                return Err("Mode index out of range!".into());
            }

            self.mode_id = index;
            Ok(&self.modes[self.mode_id])
        }
    // 

    // Remote
        fn add_remote(&mut self, remote : Box<dyn PushRemote>) {
            self.remotes.push(remote)
        }

        fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote>> {
            &self.remotes
        }

        fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote>> {
            &mut self.remotes
        }
    //
}