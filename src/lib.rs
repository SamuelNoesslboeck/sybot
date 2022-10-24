use std::{fs, f32::consts::PI};

use glam::Vec3;
use serde::{Serialize, Deserialize, ser::SerializeTuple, de::Visitor};
use serde_json::{Value, json};

// Structures
    #[derive(Clone)]
    pub struct PVec3
    {
        pub v : Vec3,
        pub l : f32
    }
    
    #[derive(Serialize, Deserialize)]
    struct PVec3Helper
    {
        pub x : f32,
        pub y : f32,
        pub z : f32
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct VecTable
    {
        pub a_b : PVec3,
        pub a_1 : PVec3,
        pub a_2 : PVec3,
        pub a_3 : PVec3,
        pub a_t : PVec3,

        pub c_1 : PVec3,
        pub c_2 : PVec3,

        pub c_m1a : PVec3,
        pub c_m1b : PVec3,
        pub c_m2a : PVec3,
        pub c_m2b : PVec3
    }   

    #[derive(Serialize, Deserialize, Clone)]
    pub struct AngTable
    {
        pub gamma_1 : f32,
        pub gamma_2 : f32,
        pub gamma_3 : f32,

        pub delta_a1 : f32,
        pub delta_a2 : f32,
        pub delta_a3 : f32
    }

    pub struct PinTable 
    {

    }

    #[derive(Serialize, Deserialize)]
    pub struct SyArm
    {
        pub vecs_0 : VecTable,
        pub vecs : VecTable,
        
        pub angles_0 : AngTable,
        pub angles : AngTable
    }
//

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
    pub fn new(v : Vec3) -> Self
    {
        return PVec3 {
            v: v,
            l: v.length()
        };
    }

    pub fn angle_for_tri(&self, a : &PVec3, b : &PVec3) -> f32
    {
        return ((a.l.powi(2) + b.l.powi(2) - self.l.powi(2)) / (2.0 * a.l * b.l)).acos()
    }
}

impl AngTable 
{
    pub fn from_vecs(vt : &VecTable) -> Self 
    {
        let vsub_1 = PVec3::new(vt.a_1.v + vt.c_m1b.v);
        let vsub_2 = PVec3::new(vt.a_1.v - vt.c_m2a.v);
        let vsub_3 = PVec3::new(vt.a_2.v + vt.c_m2b.v);
        let vsub_4 = PVec3::new(vt.a_2.v + vt.a_3.v);

        let delta_a1 = vt.c_m1b.angle_for_tri(&vt.a_1, &vsub_1);
        let delta_a2 = vt.c_m2a.angle_for_tri(&vt.a_1, &vsub_2);
        let delta_a3 = vt.c_m2b.angle_for_tri(&vt.a_2, &vsub_3);

        let gamma_1_ = vt.c_1.angle_for_tri(&vt.c_m1a, &vsub_1);
        let gamma_1 = PI - gamma_1_;

        let gamma_2_ = vt.c_2.angle_for_tri(&vsub_2, &vsub_3);
        let gamma_2 = gamma_2_ - PI;

        let gamma_3_ = vsub_4.angle_for_tri(&vt.a_2, &vt.a_3);
        let gamma_3 = PI - gamma_3_; 

        return Self {
            gamma_1: gamma_1,
            gamma_2: gamma_2,
            gamma_3: gamma_3,

            delta_a1: delta_a1,
            delta_a2: delta_a2,
            delta_a3: delta_a3
        }; 
    }
}

impl SyArm
{
    pub fn from_dim(vec_0 : VecTable) -> Self
    {
        
    }

    pub fn load_0(p : &str) -> Self
    {
        let vecs_0 : VecTable = serde_json::from_str(
            fs::read_to_string(p).unwrap().as_str()
        ).unwrap();

        return Self::from_dim(vecs_0);
    }   

    pub fn save_0(&self, p : &str) 
    {
        fs::write(p, 
            serde_json::to_string(&self.angles_0).unwrap()
        ).unwrap();
    }

    pub fn reset(&mut self)
    {
        self.vecs = self.vecs_0.clone();
        self.angles = self.angles_0.clone();
    }
}