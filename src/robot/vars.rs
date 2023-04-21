use glam::Vec3;

#[derive(Clone, Debug)]
pub struct RobotVars {
    pub load : f32,
    
    pub f_speed : f32,

    pub decos : Vec<f32>,
    pub point : Vec3
}

impl Default for RobotVars {
    fn default() -> Self {
        Self {
            load: Default::default(),
            f_speed: 1.0,

            decos: Default::default(),
            point: Default::default()
        }
    }
}