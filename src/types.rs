/// Library types
pub use glam::Vec3;
pub use glam::Mat3;

// Types 
pub type SyArmResult<T> = Result<T, SyArmError>;

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

impl std::fmt::Display for Gammas {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {}, {}, {})", self.0, self.1, self.2, self.3)
    }
}

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

#[derive(Copy, Clone, Debug)]
pub enum ErrType {
    None,
    
    // Movements
    OutOfRange,
    BadPins
}

#[derive(Debug)]
pub struct SyArmError 
{
    pub msg : String,
    pub err_type : ErrType
}

impl std::fmt::Display for SyArmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[SyArm-Error {}] {}", self.err_type as u64, self.msg)
    }
}

impl std::error::Error for SyArmError {
    
}

impl SyArmError {
    pub fn new_simple(err_type : ErrType) -> Self {
        Self {
            err_type: err_type,
            msg: String::new()
        }
    }

    pub fn new(msg : &str, err_type : ErrType) -> Self {
        Self {
            err_type: err_type,
            msg: String::from(msg)
        }
    }
}