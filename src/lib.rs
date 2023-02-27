#![crate_name = "sybot_lib"]
//! # SyBot Library
//! 
//! Control and calculation library various robots

use stepper_lib::{Component, ComponentGroup, Omega, Gamma, Delta};

// Module decleration
    mod arm;
    pub use arm::SyArm;

    pub mod intpr;
    pub use intpr::init_intpr;

    mod omats;
    pub use omats::Syomat;

    mod robot;
    pub use robot::*;

    pub mod server;

    pub mod types;

    #[cfg(test)]
    mod tests;
//

// Public imports
pub use stepper_lib::{JsonConfig, MachineConfig};
pub use stepper_lib::gcode::Interpreter;

// Basic robot
pub struct BasicRobot<const N : usize, const D : usize, const A : usize>
{
    pub conf : Option<JsonConfig>,
    pub mach : MachineConfig<N, D, A>,

    pub vars : RobotVars,

    // Controls
    pub comps : [Box<dyn Component>; N],

    tool_id : usize
}

impl<const N : usize, const D : usize, const A : usize> ConfRobot<N> for BasicRobot<N, D, A>
{
    // Conf
        fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error> {
            let (mach, comps) = conf.get_machine()?;

            Ok(Self { 
                conf: Some(conf), 
                mach: mach,
                comps: comps,

                vars: RobotVars::default(),

                tool_id: 0
            })
        }

        #[inline]
        fn json_conf(&self) -> &Option<JsonConfig> {
            &self.conf
        }
    //

    // Data 
        #[inline]
        fn comps(&self) -> &dyn ComponentGroup<N> {
            &self.comps
        }

        #[inline]
        fn comps_mut(&mut self) -> &mut dyn ComponentGroup<N> {
            &mut self.comps
        }

        #[inline]
        fn vars(&self) -> &RobotVars {
            &self.vars
        }

        #[inline]
        fn max_vels(&self) -> &[Omega; N] {
            &self.mach.vels
        }

        #[inline]
        fn meas_dists(&self) -> &[Delta; N] {
            &self.mach.meas_dist
        }

        #[inline]
        fn home_pos(&self) -> &[Gamma; N] {
            &self.mach.home
        }

        #[inline]
        fn anchor(&self) -> &glam::Vec3 {
            &self.mach.anchor
        }
    //
}