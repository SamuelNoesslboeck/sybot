#![crate_name = "sybot_lib"]
//! # SyBot Library
//! 
//! Control and calculation library various robots

extern crate alloc;

use colored::Colorize;

use stepper_lib::{Tool, SyncComp, SyncCompGroup};
use stepper_lib::units::*;

// Module decleration
    /// I/O of configuration files to parse whole component groups out of text
    pub mod conf;
    pub use conf::{JsonConfig, MachineConfig};

    /// Resources required to process and generate G-Code
    pub mod gcode;

    pub mod intpr;
    pub use intpr::init_intpr;

    mod robot;
    pub use robot::*;

    pub mod server;

    #[cfg(test)]
    mod tests;
//

// Basic robot
pub struct BasicRobot<const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> {
    conf : Option<JsonConfig>,
    mach : MachineConfig<COMP, DIM, ROT>,

    vars : RobotVars<DECO>,

    // Controls
    comps : [Box<dyn SyncComp>; COMP],

    tool_id : usize
}

impl<const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> BasicRobot<COMP, DECO, DIM, ROT> {
    #[cfg(feature = "dbg-funcs")]
    pub fn print_conf_header(&self) {
        if let Some(conf) = &self.conf {
            println!("{}", format!("[{}]", conf.name).bright_blue().bold());
            println!("| {} {}", "Version:".bold(), conf.conf_version.italic().truecolor(0xEA, 0x8C, 0x43));

            if let Some(author) = &conf.author {
                println!("| {} {}", "Author:".bold(), author.italic().yellow());
            }

            println!("|");
            println!("| {}", "[Components]".bright_blue().bold());
            for i in 0 .. COMP {
                println!("| | {}: {}", conf.comps[i].name, format!("\"{}\"", conf.comps[i].type_name.split("::").last().unwrap()).green());
            }

            println!("|");
            println!("| {}", "[Tools]".bright_blue().bold());
            for i in 0 .. conf.tools.len() {
                println!("| | {}: {}", conf.tools[i].name, format!("\"{}\"", conf.tools[i].type_name.split("::").last().unwrap()).green());
            }
        }
    }
}

impl<const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> ConfRobot<COMP, DECO, DIM, ROT> for BasicRobot<COMP, DECO, DIM, ROT> {
    // Setup
        fn setup(&mut self) {
            self.comps.setup();
        }

        fn setup_async(&mut self) {
            self.comps.setup_async();
        }
    // 

    // Conf
        fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error> {
            let mach = conf.get_machine()?;
            let comps = conf.get_async_comps()?;

            Ok(Self { 
                conf: Some(conf), 
                mach,
                comps,

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
        fn comps(&self) -> &dyn SyncCompGroup<dyn SyncComp, COMP> {
            &self.comps
        }

        #[inline]
        fn comps_mut(&mut self) -> &mut dyn SyncCompGroup<dyn SyncComp, COMP> {
            &mut self.comps
        }

        #[inline]
        fn vars(&self) -> &RobotVars<DECO> {
            &self.vars
        }

        fn mach(&self) -> &MachineConfig<COMP, DIM, ROT> {
            &self.mach
        }

        #[inline]
        fn max_vels(&self) -> &[Omega; COMP] {
            &self.mach.vels
        }

        #[inline]
        fn meas_deltas(&self) -> &[Delta; COMP] {
            &self.mach.meas_dist
        }

        #[inline]
        fn home_pos(&self) -> &[Gamma; COMP] {
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

        #[inline]
        fn set_tool_id(&mut self, tool_id : usize) -> Option<&mut Box<dyn Tool + std::marker::Send>> {
            if tool_id < self.mach.tools.len() {
                if let Some(t) = self.get_tool_mut() {
                    t.dismount();
                }

                self.tool_id = tool_id;

                return match self.get_tool_mut() {
                    Some(t) => {
                        t.mount(); 
                        Some(t)
                    },
                    None => None
                }
            }

            None
        }

        fn gamma_tool(&self) -> Option<Gamma> {
            if let Some(any_tool) = self.get_tool() {
                if let Some(tool) = any_tool.axis_tool() {
                    return Some(tool.gamma()) 
                }
            }

            None
        }

        // Actions
        #[inline]
        fn activate_tool(&mut self) -> Option<bool> {
            if let Some(any_tool) = self.get_tool_mut() {
                if let Some(tool) = any_tool.simple_tool_mut() {
                    tool.activate();

                    return Some(tool.is_active())
                }
            }

            None
        }

        #[inline]
        fn activate_spindle(&mut self, cw : bool) -> Option<bool> {
            if let Some(any_tool) = self.get_tool_mut() {
                if let Some(spindle) = any_tool.spindle_tool_mut() {
                    spindle.activate(cw);

                    return spindle.is_active()
                }
            }
            
            None
        }

        #[inline]
        fn deactivate_tool(&mut self) -> Option<bool> {
            if let Some(any_tool) = self.get_tool_mut() {
                if let Some(tool) = any_tool.simple_tool_mut() {
                    tool.deactivate();

                    return Some(tool.is_active())
                }

                if let Some(spindle) = any_tool.spindle_tool_mut() {
                    spindle.deactivate();

                    return spindle.is_active()
                }
            }

            None
        }

        #[inline]
        fn rotate_tool_abs(&mut self, gamma : Gamma) -> Option<Gamma> {
            if let Some(any_tool) = self.get_tool_mut() {
                if let Some(tool) = any_tool.axis_tool_mut() {
                    tool.rotate_abs(gamma);

                    return Some(gamma)
                }
            }

            None
        }
    //
}