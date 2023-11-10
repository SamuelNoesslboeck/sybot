use glam::{Vec3, Mat3};
use syact::units::*;

use crate::rcs::PointRef;

#[derive(Debug)]
pub enum Movement {
    Rotation,
    Linear(Vec3)
}

#[derive(Debug)]
pub struct KinElement {
    movement : Movement,
    point_0 : PointRef,

    _phi : Phi,
    _point : PointRef
}

impl KinElement {
    pub fn new(movement : Movement, point_0 : PointRef) -> Self {
        Self {
            movement,
            _point: point_0.clone_no_ref(),

            _phi: Phi::ZERO,
            point_0
        }
    }

    // Accessors
        pub fn phi(&self) -> Phi {
            self._phi
        }

        pub fn point(&self) -> &PointRef {
            &self._point
        }
    // 

    #[inline]
    pub fn pos(&self) -> Vec3 {
        self._point.pos()    
    }

    pub fn update(&mut self, phi : Phi) -> Result<(), crate::Error> {
        self._phi = phi;

        match self.movement {
            Movement::Rotation => {
                let mut p_ref = self._point.borrow_mut();
                let p0_ref = self.point_0.borrow();

                *p_ref.ori_mut() = *p0_ref.ori() * Mat3::from_rotation_z(phi.0);
            },
            Movement::Linear(pos) => {
                let mut p_ref = self._point.borrow_mut();
                let p0_ref = self.point_0.borrow();

                *p_ref.pos_mut() = *p0_ref.pos() + pos * phi.0;
            }
        }

        Ok(())
    }
}