use colored::Colorize;

use stepper_lib::{Tool, SyncComp, SyncCompGroup};
use stepper_lib::units::*;

use crate::Robot;
use crate::conf::{JsonConfig, MachineConfig};
use crate::remote::PushRemote;
use crate::robot::RobotVars;

/// A basic robot structure which new robot types can derive upon
pub struct BasicRobot<const C : usize> {
    conf : Option<JsonConfig>,
    mach : MachineConfig<C>,

    vars : RobotVars,

    rem : Vec<Box<dyn PushRemote<C>>>,

    // Controls
    comps : [Box<dyn SyncComp>; C],

    tool_id : usize
}

impl<const C : usize> BasicRobot<C> {
    /// Prints a brief summary of the configuration file applied to the robot
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
            for i in 0 .. C {
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

impl<const C : usize> Robot<C> for BasicRobot<C> {
    // Setup
        fn setup(&mut self) {
            self.comps.setup();
        }

        fn setup_async(&mut self) {
            self.comps.setup_async();
        }
    // 

    // Conf
        fn from_conf(conf : JsonConfig, dim : usize, rot : usize) -> Result<Self, crate::Error> {
            let mach = conf.get_machine::<C>(dim, rot)?;
            let comps = conf.get_async_comps()?;

            Ok(Self { 
                conf: Some(conf), 
                mach,
                comps,

                rem: vec![],

                vars: RobotVars::default(),

                tool_id: 0
            })
        }

        #[inline]
        fn json_conf<'a>(&'a self) -> Option<&'a JsonConfig> {
            match &self.conf {
                Some(conf) => Some(conf),
                None => None
            }
        }
    //

    // Data 
        #[inline]
        fn comps(&self) -> &dyn SyncCompGroup<dyn SyncComp, C> {
            &self.comps
        }

        #[inline]
        fn comps_mut(&mut self) -> &mut dyn SyncCompGroup<dyn SyncComp, C> {
            &mut self.comps
        }

        #[inline]
        fn vars(&self) -> &RobotVars {
            &self.vars
        }

        #[inline(always)]
        fn vars_mut(&mut self) -> &mut RobotVars {
            &mut self.vars
        }

        #[inline(always)]
        fn mach(&self) -> &MachineConfig<C> {
            &self.mach
        }

        #[inline]
        fn max_vels(&self) -> [Omega; C] {
            let mut vels = self.mach.vels.clone();

            for i in 0 .. C {
                vels[i] = vels[i] * self.vars.f_speed;
            }

            vels
        }

        #[inline]
        fn meas_deltas(&self) -> &[Delta; C] {
            &self.mach.meas_dist
        }

        #[inline]
        fn home_pos(&self) -> &[Gamma; C] {
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

    // Remotes
        fn add_remote(&mut self, remote : Box<dyn PushRemote<C> + 'static>) {
            self.rem.push(remote);
        }

        fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote<C>>> {
            &self.rem
        }

        fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote<C>>> {
            &mut self.rem
        }
    // 
}