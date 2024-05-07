use core::marker::PhantomData;

use glam::Vec3;
use syact::math::movements::DefinedActuator;
use syact::{Setup, SyncActuatorGroup};
use syact::act::Interruptor;
use syact::act::stepper::{StepperActuator, StepperActuatorGroup};
use syunit::*;

// use crate::pkg::{RobotPackage, parse_struct};
// use crate::pkg::info::AngConf;

use crate::{Robot, PushRemote, Descriptor};
use crate::config::{AngleConfig, Mode, default_modes};
use crate::rcs::Position;
use crate::robs::{Vars, Tool};

pub trait DynMeas : Interruptor + Setup + Send { }

pub struct StepperRobot<G, T, const C : usize> 
where 
    G : StepperActuatorGroup<T, C>,
    T : StepperActuator + ?Sized + 'static
{
    vars : Vars<C>,

    ang_confs : [AngleConfig; C],
    comps : G,

    tools : Vec<Box<dyn Tool>>,
    tool_id : Option<usize>,

    modes : Vec<Mode>,
    mode_id : usize,

    remotes : Vec<Box<dyn PushRemote>>,

    __pd : PhantomData<T>
}

impl<G, T, const C : usize> StepperRobot<G, T, C>
where 
    G : StepperActuatorGroup<T, C>,
    T : StepperActuator + ?Sized + 'static
{
    pub fn new(ang_confs : [AngleConfig; C], comps : G, tools : Vec<Box<dyn Tool>>) -> Self {
        Self {
            vars: Vars::default(),

            ang_confs,
            comps,
            
            tools,
            tool_id: None,

            modes: Vec::from(default_modes()),
            mode_id: 0,

            remotes: Vec::new(),

            __pd : PhantomData::default()
        }
    }
}

impl<G, T, const C : usize> Setup for StepperRobot<G, T, C> 
where 
    G : StepperActuatorGroup<T, C>,
    T : StepperActuator + DefinedActuator + ?Sized + 'static
{
    fn setup(&mut self) -> Result<(), syact::Error> {
        self.comps_mut().setup()
    }
}

impl<G, T, const C : usize> Robot<G, T, C> for StepperRobot<G, T, C> 
where 
    G : StepperActuatorGroup<T, C>,
    T : StepperActuator + DefinedActuator + ?Sized + 'static
{
    // Data
        #[inline]
        fn ang_confs<'a>(&'a self) -> &[AngleConfig; C] {
            &self.ang_confs
        }

        #[inline]
        fn comps<'a>(&'a self) -> &'a G {
            &self.comps
        }

        #[inline]
        fn comps_mut<'a>(&'a mut self) -> &'a mut G {
            &mut self.comps
        }
        
        #[inline]
        fn vars<'a>(&'a self) -> &'a Vars<C> {
            &self.vars
        }
        
        #[inline]
        fn vars_mut<'a>(&'a mut self) -> &'a mut Vars<C> {
            &mut self.vars
        }
    //

    // Movement
        fn move_p_sync<D : Descriptor<C>>(&mut self, desc : &mut D, p : Position, speed_f : Factor) -> Result<(), crate::Error> {
            let phis = desc.phis_for_pos(p)?;
            self.move_abs_j_sync(
                phis,
                speed_f
            )
        }
    //

    // Complex movement
        #[allow(unused)]
        fn move_j(&mut self, deltas : [Delta; C], gen_speed_f : Factor) -> Result<(), crate::Error> {
            let gamma_0 = self.gammas();
            let gamma_t = add_unit_arrays(gamma_0, deltas);
            let speed_f = syact::math::movements::ptp_speed_factors(
                self.comps_mut(), gamma_0, gamma_t, gen_speed_f
            );
            <G as SyncActuatorGroup<T, C>>::drive_rel_async(self.comps_mut(), deltas, speed_f)
        }

        #[allow(unused)]
        fn move_abs_j(&mut self, phis : [Phi; C], gen_speed_f : Factor) -> Result<(), crate::Error> {
            let gamma_0 = self.gammas();
            let gamma_t = self.gammas_from_phis(phis);
            let speed_f = syact::math::movements::ptp_speed_factors(
                self.comps_mut(), gamma_0, gamma_t, gen_speed_f
            );
            <G as SyncActuatorGroup<T, C>>::drive_abs_async(self.comps_mut(), gamma_t, speed_f)
        }

        #[allow(unused)]
        fn move_l<D : Descriptor<C>>(&mut self, desc : &mut D, distance : Vec3, accuracy : f32, speed : Velocity) -> Result<(), crate::Error> {
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

            // let mut builder = self.comps.create_path_builder([Velocity::ZERO; C]);
            // builder.generate(&mut tstack, dstack.as_slice(), [Some(Velocity::ZERO); C]);

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
            //     self.comps.drive_nodes(&last, [Velocity::ZERO; C], &mut corr)?;
            // }

            // dbg!(corr, t_err);

            // println!("Done");

            Ok(())
        }

        fn move_abs_l<D : Descriptor<C>>(&mut self, desc : &mut D, pos : Vec3, accuracy : f32, speed : Velocity) -> Result<(), crate::Error> {
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