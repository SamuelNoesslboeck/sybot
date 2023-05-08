use glam::Vec3;

pub trait RobPos {
    fn pos<'a>(&'a self) -> &'a Vec3;
    fn ori<'a>(&'a self) -> &'a Vec3;
}

pub struct WorldObject {
    pub points : Vec<Vec3>
}