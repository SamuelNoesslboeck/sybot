use glam::Vec3;
use syact::{SyncCompGroup, SyncComp};
use syact::units::*;

use crate::Robot;
use crate::conf::AxisConf;
use crate::rcs::{PointRef, Position, WorldObj};

// ####################
// #    SUBMODULES    #
// ####################
    mod kin;
// 

/// # `Descriptor` trait
/// 
/// 
pub trait Descriptor<T : SyncCompGroup<dyn SyncComp, C>, const C : usize> {
    // Axis conf
        fn aconf<'a>(&'a self) -> &'a dyn AxisConf;

        fn aconf_mut<'a>(&'a mut self) -> &'a mut dyn AxisConf;
    //

    // Events
        fn update(&mut self, rob : &mut dyn Robot<T, C>, phis : &[Phi; C]) -> Result<(), crate::Error>;
    // 

    // Calculation
        fn convert_pos(&self, rob : &dyn Robot<T, C>, pos : Position) -> Result<[Phi; C], crate::Error>;
    //

    // World object
        fn wobj<'a>(&'a self) -> &'a WorldObj;

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj;

        fn current_tcp(&self) -> &PointRef;

        /// Create a Vec3 from optional coordinates 
        fn cache_tcp(&self, x_opt : Option<f32>, y_opt : Option<f32>, z_opt : Option<f32>) -> Vec3;
    // 
}