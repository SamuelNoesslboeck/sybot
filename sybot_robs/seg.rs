use glam::{Vec3, Mat3};
use stepper_lib::units::*;

use sybot_rcs::{PointRef, Position, Point};

#[derive(Debug)]
pub enum SegmentMovement {
    Rotation,
    Linear(Vec3)
}

#[derive(Debug)]
pub struct Segment {
    phi : Phi,
    movement : SegmentMovement,

    pub point_0 : PointRef,
    pub point : PointRef
}

impl Segment {
    pub fn update(&mut self, phi : Phi) -> Result<(), crate::Error> {
        self.phi = phi;

        match self.movement {
            SegmentMovement::Rotation => {
                let mut p_ref = self.point.borrow_mut();
                let p0_ref = self.point_0.borrow();

                *p_ref.ori_mut() = *p0_ref.ori() * Mat3::from_rotation_z(phi.0);
            },
            SegmentMovement::Linear(pos) => {
                let mut p_ref = self.point.borrow_mut();
                let p0_ref = self.point_0.borrow();

                *p_ref.pos_mut() = *p0_ref.pos() + pos * phi.0;
            }
        }

        Ok(())
    }
}

pub trait SegmentChain<const C : usize> : core::fmt::Debug {
    fn segments<'a>(&'a self) -> &'a [Segment; C]; 

    fn segments_mut<'a>(&'a mut self) -> &'a mut [Segment; C];

    fn calculate_end(&self, mut pos_0 : Position) -> Position {
        let segments = self.segments(); 

        for i in 1 ..= C {
            let index = C - i;
            let point = segments[index].point.borrow();
    
            pos_0.transform(*point.ori());
            pos_0.shift(*point.pos());
        }

        pos_0
    }

    // Events
        fn update(&mut self, phis : &[Phi; C]) -> Result<(), crate::Error> {
            let segments = self.segments_mut();

            for i in 0 .. C {
                segments[i].update(phis[i])?;
            }

            Ok(())
        }
    // 
}

#[derive(Debug)]
pub struct LinSegmentChain<const C : usize> {
    pub segments : [Segment; C]
}

impl<const C : usize> SegmentChain<C> for LinSegmentChain<C> {
    fn segments<'a>(&'a self) -> &'a [Segment; C] {
        &self.segments
    }

    fn segments_mut<'a>(&'a mut self) -> &'a mut [Segment; C] {
        &mut self.segments
    }
}