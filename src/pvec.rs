use glam::Vec3;
use serde::{Serialize, Deserialize, ser::SerializeTuple};

/// A wrapper struct for vector calculations with constant-length vectors
#[derive(Clone)]
pub struct PVec3
{
    pub v : Vec3,
    pub l : f32
}

/// Helper struct for JSON-Parsing
#[derive(Serialize, Deserialize)]
struct PVec3Helper
{
    pub x : f32,
    pub y : f32,
    pub z : f32
}

impl Serialize for PVec3 
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer 
    {
        let mut tup = serializer.serialize_tuple(3)?;
        tup.serialize_element(&self.v.x)?;
        tup.serialize_element(&self.v.y)?;
        tup.serialize_element(&self.v.z)?;
        return tup.end();
    }
}

impl<'de> Deserialize<'de> for PVec3
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> 
    {
        return Deserialize::deserialize(deserializer).map(|PVec3Helper {x, y, z}| { 
            PVec3 { 
                v: Vec3::new(x, y, z),
                l: (x.powi(2) + y.powi(2) + z.powi(2)).powf(0.5)
            }
        });
    }
}

impl PVec3 
{
    pub fn new(v : Vec3) -> Self {
        return PVec3 {
            v,
            l: v.length()
        };
    }

    pub fn angle_for_tri(&self, a : &PVec3, b : &PVec3) -> f32 {
        return ((a.l.powi(2) + b.l.powi(2) - self.l.powi(2)) / (2.0 * a.l * b.l)).acos()
    }

    pub fn as_x(&mut self) -> &Self {
        self.v = Vec3::new(self.l, 0.0, 0.0);
        return self;
    }

    pub fn into_x(&self) -> Self {
        return PVec3 { 
            v: Vec3::new(self.l, 0.0, 0.0), 
            l: self.l 
        };
    }

    pub fn into_y(&self) -> Self {
        return PVec3 { 
            v: Vec3::new(0.0, self.l, 0.0), 
            l: self.l 
        };
    }
}