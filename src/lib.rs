#![crate_name = "sybot_lib"]
//! # SyBot Library
//! 
//! Control and calculation library various robots

use stepper_lib::{Component, ComponentGroup, Omega, Gamma, Delta, Tool};

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
pub struct BasicRobot<const N : usize, const DECO : usize, const DIM : usize, const ROT : usize>
{
    pub conf : Option<JsonConfig>,
    pub mach : MachineConfig<N, DIM, ROT>,

    pub vars : RobotVars<DECO>,

    // Controls
    pub comps : [Box<dyn Component>; N],

    tool_id : usize
}

impl<const N : usize, const DECO : usize, const DIM : usize, const ROT : usize> ConfRobot<N, DECO, DIM, ROT> for BasicRobot<N, DECO, DIM, ROT>
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
        fn vars(&self) -> &RobotVars<DECO> {
            &self.vars
        }

        fn mach(&self) -> &MachineConfig<N, DIM, ROT> {
            &self.mach
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
    
    // Tools
        /// Returns the current tool that is being used by the robot
        #[inline]
        fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>> {
            self.mach.tools.get(self.tool_id)
        }

        #[inline]
        fn get_tool_mut(&mut self) -> Option<&mut Box<dyn Tool + std::marker::Send>> {
            self.mach.tools.get_mut(self.tool_id)
        }

        #[inline]
        fn get_tools(&self) -> &Vec<Box<dyn Tool + std::marker::Send>> {
            &self.mach.tools
        }

        fn set_tool_id(&mut self, tool_id : usize) {
            if tool_id < self.mach.tools.len() {
                self.tool_id = tool_id;
            }

            // TODO: Report error in tool selection
        }

        // Actions
        fn activate_tool(&mut self) {
            if let Some(any_tool) = self.get_tool_mut() {
                if let Some(tool) = any_tool.simple_tool_mut() {
                    tool.activate();
                }
            }
        }

        fn activate_spindle(&mut self, cw : bool) {
            if let Some(any_tool) = self.get_tool_mut() {
                if let Some(spindle) = any_tool.spindle_tool_mut() {
                    spindle.activate(cw);
                }
            }
        }

        fn deactivate_tool(&mut self) {
            if let Some(any_tool) = self.get_tool_mut() {
                if let Some(tool) = any_tool.simple_tool_mut() {
                    tool.deactivate();
                }

                if let Some(spindle) = any_tool.spindle_tool_mut() {
                    spindle.deactivate();
                }
            }
        }
    //
}