/// Library types
pub use glam::Vec3;
pub use glam::Mat3;

/// Exchange tuple type for directional and positional vectors of the robot 
#[derive(Clone, Copy, Debug)]
pub struct CylVectors(
    /// First cylinder for first segment \
    /// ( Direction, Position )
    pub (Vec3, Vec3),    
    /// Second cylinder for first segment \
    /// ( Direction, Position )
    pub (Vec3, Vec3),       
    /// Second cylinder for second segment \
    /// ( Direction, Position )
    pub (Vec3, Vec3)        
);

/// Exchange type type for forces and torques acting upon the robot
#[derive(Clone, Copy, Debug)]
pub struct Forces( 
    /// Base torque [Nm]
    pub f32,
    /// First cylinder force [N]
    pub f32,
    /// Second cylinder force [N]
    pub f32,
    /// Third join torque [Nm]
    pub f32
);

#[derive(Clone, Copy, Debug)]
pub struct Gammas( 
    /// Base
    pub f32,
    pub f32,
    pub f32,
    pub f32
);

#[derive(Clone, Copy, Debug)]
pub struct Inertias( 
    /// Base
    pub f32,
    pub f32,
    pub f32,
    pub f32
);

#[derive(Clone, Copy, Debug)]
pub struct Phis( 
    /// Base
    pub f32,
    pub f32,
    pub f32,
    pub f32
);

#[derive(Clone, Copy, Debug)]
pub struct Points( 
    /// Base
    pub Vec3,
    pub Vec3,
    pub Vec3,
    pub Vec3
);

#[derive(Clone, Copy, Debug)]
pub struct Vectors( 
    /// Base
    pub Vec3,
    pub Vec3,
    pub Vec3,
    pub Vec3
);
