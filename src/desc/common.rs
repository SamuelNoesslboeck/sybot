use crate::{Descriptor, Robot};
use crate::desc::{SerialKinematic, Kinematic, KinElement, Movement};
use crate::rcs::{Point, Position, PointRef, WorldObj};

use glam::Vec3;
use syact::{SyncComp, SyncCompGroup};
use syact::units::*;

pub struct LinearXYZDescriptor {
    _kinematic : SerialKinematic<3>,
    _world_obj : WorldObj,

    __axis_config : ()
}

impl Descriptor<3> for LinearXYZDescriptor {
    // Types
        type AxisConfig = ();
        type Kinematic = SerialKinematic<3>;
    // 

    // Axis config
        fn axis_config(&self) -> &Self::AxisConfig {
            &self.__axis_config
        }

        fn axis_config_mut(&mut self) -> &mut Self::AxisConfig {
            &mut self.__axis_config
        }
    // 

    // Calculation
        fn phis_for_pos(&self, pos : Position) -> Result<[Phi; 3], crate::Error> {
            Ok([
                Phi(pos.x()),
                Phi(pos.y()),
                Phi(pos.z())
            ])
        }
    //

    // Kinematic
        fn kinematic(&self) -> &Self::Kinematic {
            &self._kinematic
        }

        fn kinematic_mut(&mut self) -> &mut Self::Kinematic {
            &mut self._kinematic
        }
    // 

    // World object
        fn world_obj(&self) -> &WorldObj {
            &self._world_obj
        }

        fn world_obj_mut(&mut self) -> &mut WorldObj {
            &mut self._world_obj
        }

        fn tcp(&self) -> &PointRef {
            self._kinematic.tcp()
        }
    // 

    // Events
        fn update<R, G, T>(&mut self, _rob : &mut R, phis : &[Phi; 3]) -> Result<(), crate::Error>
        where
            R : Robot<G, T, 3>,
            G : SyncCompGroup<T, 3>,
            T : SyncComp + ?Sized + 'static
        {
            self._kinematic.update(phis)
        }
    // 
}

impl LinearXYZDescriptor {
    pub fn new() -> Self {
        let wobj = WorldObj::zero()
            .add_point_inline("x", PointRef::new(
                WorldObj::zero()
                    .add_point_inline("y", PointRef::new(
                        WorldObj::zero()
                            .add_point_inline("z", PointRef::new(Position::zero()))
                    ))
            ));


        Self {
            _kinematic: SerialKinematic::new([
                KinElement::new(Movement::Linear(Vec3::X), wobj.point("x").unwrap()),
                KinElement::new(Movement::Linear(Vec3::Y), wobj.point("x/y").unwrap()),
                KinElement::new(Movement::Linear(Vec3::Z), wobj.point("x/y/z").unwrap())
            ]),
            _world_obj: wobj,
            __axis_config: ()
        }
    }
}