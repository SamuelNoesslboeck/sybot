use glam::{Vec2, Vec3};
use syunit::metric::Millimeters;

use crate::Descriptor;

pub struct LinDesc2 { }

impl Descriptor for LinDesc2 {
    type SysCoord = Vec2;
    type RobCoord = (Millimeters, Millimeters);
    type Error = ();    // TODO: Add error

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Self::SysCoord {
        Vec2::new(rob.0.into(), rob.1.into())
    }

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, Self::Error> {
        Ok((Millimeters(sys.x), Millimeters(sys.y)))
    }
}

pub struct LinDesc3 { }

impl Descriptor for LinDesc3 {
    type SysCoord = Vec3;
    type RobCoord = (Millimeters, Millimeters, Millimeters);
    type Error = ();

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Self::SysCoord {
        Vec3::new(rob.0.into(), rob.1.into(), rob.2.into())
    }

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, Self::Error> {
        Ok((Millimeters(sys.x), Millimeters(sys.y), Millimeters(sys.z)))
    }
}
