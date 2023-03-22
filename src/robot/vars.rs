use glam::Vec3;

#[derive(Clone, Debug)]
pub struct RobotVars<const DECO : usize> {
    pub load : f32,
    
    pub f_speed : f32,

    pub decos : [f32; DECO],
    pub point : Vec3
}

impl<const DECO : usize> Default for RobotVars<DECO> {
    fn default() -> Self {
        Self {
            load: Default::default(),
            f_speed: 1.0,

            decos: [Default::default(); DECO],
            point: Default::default()
        }
    }
}