use glam::{Vec3, Mat3};
use syact::units::*;

// use crate::pkg::info::{SegmentInfo, SegmentMovementInfo};
use crate::rcs::{PointRef, Point};

#[derive(Debug)]
pub enum Movement {
    Rotation,
    Linear(Vec3)
}

// TODO: Move to pkg
// impl From<SegmentMovementInfo> for SegmentMovement {
//     fn from(info : SegmentMovementInfo) -> Self {
//         match info {
//             SegmentMovementInfo::Rotation => SegmentMovement::Rotation,
//             SegmentMovementInfo::Linear(fvec) => SegmentMovement::Linear(Vec3::from(fvec))
//         }
//     }
// }

#[derive(Debug)]
pub struct KinElement {
    movement : Movement,
    point_0 : PointRef,

    phi : Phi,
    pub point : PointRef
}

impl KinElement {
    // pub fn new(point : PointRef, info : &SegmentInfo) -> Self {
    //     Self {
    //         phi: Phi::ZERO,
    //         movement: SegmentMovement::from(info.movement),

    //         point_0 : point.clone_no_ref(),
    //         point
    //     }
    // }

    pub fn new(movement : Movement, point_0 : PointRef) -> Self {
        Self {
            movement,
            point: point_0.clone_no_ref(),

            phi: Phi::ZERO,
            point_0
        }
    }

    #[inline]
    pub fn pos(&self) -> Vec3 {
        self.point.pos()    
    }

    pub fn update(&mut self, phi : Phi) -> Result<(), crate::Error> {
        self.phi = phi;

        match self.movement {
            Movement::Rotation => {
                let mut p_ref = self.point.borrow_mut();
                let p0_ref = self.point_0.borrow();

                *p_ref.ori_mut() = *p0_ref.ori() * Mat3::from_rotation_z(phi.0);
            },
            Movement::Linear(pos) => {
                let mut p_ref = self.point.borrow_mut();
                let p0_ref = self.point_0.borrow();

                *p_ref.pos_mut() = *p0_ref.pos() + pos * phi.0;
            }
        }

        Ok(())
    }
}