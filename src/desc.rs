use syact::{SyncActuatorGroup, SyncActuator};
use syunit::*;

use crate::Robot;
use crate::config::AxisConfig;
use crate::rcs::{PointRef, Position, WorldObj};

// ####################
// #    SUBMODULES    #
// ####################
    /// A set of commonly used descriptors
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
        /// The type of `AxisConfig` that should be used to define movements
        type AxisConfig : AxisConfig;
        /// The kinematic system used
        type Kinematic : Kinematic<C>;
    // 

    // Axis config
        /// Returns a reference to the current `AxisConfig` used
        fn axis_config(&self) -> &Self::AxisConfig;
        
        /// Returns a mutable reference to the current `AxisConfig` used
        fn axis_config_mut(&mut self) -> &mut Self::AxisConfig;
    //

    // Calculation
        /// Returns the `Phi` values required to reach a certain position
        fn phis_for_pos(&self, pos : Position) -> Result<[Phi; C], crate::Error>;
    //

    // Kinematic
        /// Returns a reference to the kinematic system used
        fn kinematic(&self) -> &Self::Kinematic;

        /// Returns a mutable reference to the kinematic system used
        fn kinematic_mut(&mut self) -> &mut Self::Kinematic;
    // 

    // World object
        /// Returns a reference to the `WorldObj` used
        fn world_obj(&self) -> &WorldObj;

        /// Returns a mutable reference to the `WorldObj` used
        fn world_obj_mut(&mut self) -> &mut WorldObj;

        /// Returns a reference to the TCP (Tool-Center-Point)
        fn tcp(&self) -> &PointRef;
    // 

    // Events
        /// Updates the descriptor and its coordinate system with the given `Phi` values
        fn update<R, G, T>(&mut self, rob : &mut R, phis : &[Phi; C]) -> Result<(), crate::Error>
        where
            R : Robot<G, T, C>,
            G : SyncActuatorGroup<T, C>,
            T : SyncActuator + ?Sized + 'static;
    // 
}