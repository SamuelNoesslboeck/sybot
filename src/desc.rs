use syact::{SyncActuatorGroup, SyncActuator};
use syact::units::*;

use crate::Robot;
use crate::config::AxisConfig;
use crate::rcs::{PointRef, Position, WorldObj};

// ####################
// #    SUBMODULES    #
// ####################
    pub mod common;

    mod elem;
    pub use elem::{KinElement, Movement, Rot};

    mod kin;
    pub use kin::{Kinematic, SerialKinematic};
// 

/// # `Descriptor` trait
/// 
/// 
pub trait Descriptor<const C : usize> {
    // Types
        type AxisConfig : AxisConfig;
        type Kinematic : Kinematic<C>;
    // 

    // Axis config
        fn axis_config(&self) -> &Self::AxisConfig;

        fn axis_config_mut(&mut self) -> &mut Self::AxisConfig;
    //

    // Calculation
        fn phis_for_pos(&self, pos : Position) -> Result<[Phi; C], crate::Error>;

    //

    // Kinematic
        fn kinematic(&self) -> &Self::Kinematic;

        fn kinematic_mut(&mut self) -> &mut Self::Kinematic;
    // 

    // World object
        fn world_obj(&self) -> &WorldObj;

        fn world_obj_mut(&mut self) -> &mut WorldObj;

        fn tcp(&self) -> &PointRef;
    // 

    // Events
        fn update<R, G, T>(&mut self, rob : &mut R, phis : &[Phi; C]) -> Result<(), crate::Error>
        where
            R : Robot<G, T, C>,
            G : SyncActuatorGroup<T, C>,
            T : SyncActuator + ?Sized + 'static;
    // 
}