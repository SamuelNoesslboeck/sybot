use glam::Vec3;
use syact::{SyncCompGroup, SyncComp};
use syact::units::*;

use crate::Robot;
use crate::conf::AxisConf;
use crate::rcs::{PointRef, Position, WorldObj};

// ####################
// #    SUBMODULES    #
// ####################
    mod elem;
    pub use elem::{KinElement, Movement, Rot};

    mod kin;
    pub use kin::{Kinematic, SerialKinematic};
// 

/// # `Descriptor` trait
/// 
/// 
pub trait Descriptor<G : SyncCompGroup<T, C>, T : SyncComp + ?Sized + 'static, const C : usize> {
    // Axis conf
        fn aconf(&self) -> &dyn AxisConf;

        fn aconf_mut(&mut self) -> &mut dyn AxisConf;
    //

    // Events
        fn update(&mut self, rob : &mut dyn Robot<G, T, C>, phis : &[Phi; C]) -> Result<(), crate::Error>;
    // 

    // Calculation
        fn convert_pos(&self, rob : &dyn Robot<G, T, C>, pos : Position) -> Result<[Phi; C], crate::Error>;
    //

    // Kinematics
        fn kin(&self) -> &dyn Kinematic<C>;

        fn kin_mut(&mut self) -> &mut dyn Kinematic<C>;
    // 

    // World object
        fn wobj(&self) -> &WorldObj;

        fn wobj_mut(&mut self) -> &mut WorldObj;

        fn current_tcp(&self) -> &PointRef;

        /// Create a Vec3 from optional coordinates 
        fn cache_tcp(&self, x_opt : Option<f32>, y_opt : Option<f32>, z_opt : Option<f32>) -> Vec3;
    // 
}