use core::marker::PhantomData;

use glam::Vec3;
use syact::{SyncActuatorGroup, Setup};
use syact::act::stepper::{StepperActuator, StepperActuatorGroup};
use syact::math::movements::DefinedActuator;
use syunit::*;
use tokio::task::JoinSet;

use crate::{Robot, PushRemote, Descriptor};
use crate::config::AngleConfig;
use crate::robs::{Vars, Tool};

/// A robot that uses stepper motors as actuators
pub struct StepperRobot<G, T, const C : usize> 
where 
    G : StepperActuatorGroup<T, C>,
    T : StepperActuator + ?Sized + 'static
{
    _vars : Vars<C>,

    _ang_confs : [AngleConfig; C],
    _comps : G,

    tools : Vec<Box<dyn Tool>>,
    tool_id : Option<usize>,

    remotes : Vec<Box<dyn PushRemote>>,

    __pd : PhantomData<T>
}

impl<G, T, const C : usize> StepperRobot<G, T, C>
where 
    G : StepperActuatorGroup<T, C>,
    T : StepperActuator + ?Sized + 'static
{
    /// Creates a new stepper robot from 
    /// - `ang_confs`: A given set of angle configurations, to convert phi into gamma values
    /// - `comps`: The components of the robot, must be a `StepperActuatorGroup`
    /// - `tools`: The set of tools equipped by the robot
    pub fn new(ang_confs : [AngleConfig; C], comps : G, tools : Vec<Box<dyn Tool>>) -> Self {
        Self {
            _vars: Vars::default(),

            _ang_confs: ang_confs,
            _comps: comps,
            
            tools,
            tool_id: None,

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
            &self._ang_confs
        }

        #[inline]
        fn comps<'a>(&'a self) -> &'a G {
            &self._comps
        }

        #[inline]
        fn comps_mut<'a>(&'a mut self) -> &'a mut G {
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
        #[allow(unused)]
        async fn move_j(&mut self, deltas : [Delta; C], gen_speed_f : Factor) -> Result<(), crate::Error> {
            let gamma_0 = self.gammas();
            let gamma_t = add_unit_arrays(gamma_0, deltas);
            let speed_f = syact::math::movements::ptp_speed_factors(
                self.comps_mut(), gamma_0, gamma_t, gen_speed_f
            );

            let mut set = JoinSet::new();

            for fut in <G as SyncActuatorGroup<T, C>>::drive_rel(self.comps_mut(), deltas, speed_f) {
                set.spawn(fut);
            }

            while let Some(res) = set.join_next().await {
                res?;
            }

            Ok(())
        }

        #[allow(unused)]
        async fn move_l<D : Descriptor<C>>(&mut self, desc : &mut D, distance : Vec3, accuracy : f32, speed : Velocity) -> Result<(), crate::Error> {
            todo!();
            Ok(())
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