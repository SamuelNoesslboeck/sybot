use glam::Vec3; 

use stepper_lib::{Tool, JsonConfig};

#[derive(Clone, Debug, Default)]
pub struct RobotVars
{
    pub load : f32,

    pub dec_angle : f32,
    pub point : Vec3
}

pub trait Robot
{
    // Configurations
        fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error>
            where
                Self: Sized;

        fn json_conf(&self) -> &JsonConfig;
    // 

    // Tools
        /// Returns the current tool that is being used by the robot
        fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>>;

        fn set_tool_id(&mut self, tool_id : usize);
    //

    // Variables
        fn get_vars(&self) -> &RobotVars;
    //
}