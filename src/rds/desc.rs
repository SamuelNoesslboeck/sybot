use glam::Vec3;
use syact::{SyncComp, SyncCompGroup};
use syact::units::*;

use crate::rcs::{PointRef, Position, WorldObj};
use crate::rds::Robot;

// Descriptors
mod basic_xyz;
pub use basic_xyz::*;

pub trait AxisConf {
    fn phis<'a>(&'a self) -> &'a [Phi];

    fn configure(&mut self, phis : Vec<Phi>) -> Result<(), crate::Error>; 
}

#[derive(Clone, Debug, Default)]
pub struct EmptyConf { }

impl AxisConf for EmptyConf {
    fn phis<'a>(&'a self) -> &'a [Phi] {
        &[]
    }

    fn configure(&mut self, _ : Vec<Phi>) -> Result<(), crate::Error> {
        Ok(())
    }
}

pub trait Descriptor<const C : usize> {
    // Axis conf
        fn aconf<'a>(&'a self) -> &'a dyn AxisConf;

        fn aconf_mut<'a>(&'a mut self) -> &'a mut dyn AxisConf;
    //

    // Events
        fn update<R : Robot<C, Comp = S, CompGroup = G>, S : SyncComp + ?Sized + 'static, G : SyncCompGroup<S, C>>(&mut self, rob : &mut R, phis : &[Phi; C]) -> Result<(), crate::Error>;
    // 

    // Calculation
        fn convert_pos<R : Robot<C, Comp = S, CompGroup = G>, S : SyncComp + ?Sized, G>(&mut self, rob : &mut R, pos : Position) -> Result<[Phi; C], crate::Error>;
    //

    // World object
        fn wobj<'a>(&'a self) -> &'a WorldObj;

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj;

        fn tcp(&self) -> &PointRef;

        /// Create a Vec3 from optional coordinates 
        fn cache_tcp(&self, x_opt : Option<f32>, y_opt : Option<f32>, z_opt : Option<f32>) -> Vec3 {
            let tcp = self.tcp().pos();

            Vec3::new( 
                x_opt.unwrap_or(tcp.x),
                y_opt.unwrap_or(tcp.y),
                z_opt.unwrap_or(tcp.z)
            )
        }
    // 
}
