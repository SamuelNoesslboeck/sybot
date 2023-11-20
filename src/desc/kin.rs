use core::ops::Index;

use syact::units::*;

use crate::rcs::{PointRef, Position, Point};
use crate::desc::KinElement;

pub trait Kinematic<const C : usize> : core::fmt::Debug {
    // Segments
        /// All the segments of the kinematic system
        fn segments(&self) -> &[KinElement; C]; 

        /// All the segments of the kinematic system
        fn segments_mut(&mut self) -> &mut [KinElement; C];

        /// The TCP (Tool-Center-Point) of the kinematic
        fn tcp<'a>(&'a self) -> &'a PointRef;

        /// The TCP (Tool-Center-Point) of the kinematic
        fn tcp_mut<'a>(&'a mut self) -> &'a mut PointRef;
    // 

    // Data
        fn phis<'a>(&'a self) -> [Phi; C] {
            let mut phis = [Phi::ZERO; C];
            for i in 0 .. C {
                phis[i] = self.segments()[i].phi();
            }
            phis
        }
    // 
    
    fn calculate_end(&self) -> Position;

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
pub struct SerialKinematic<const C : usize> {
    segments : [KinElement; C],
    tcp : PointRef
}

impl<const C : usize> SerialKinematic<C> {
    pub fn new(segments : [KinElement; C]) -> Self {
        let last = &segments[C - 1];

        Self {
            tcp: last.point().clone(),
            segments
        }
    }
}

impl<const C : usize> Index<usize> for SerialKinematic<C> {
    type Output = KinElement;

    fn index(&self, index: usize) -> &Self::Output {
        &self.segments[index]
    }
}

impl<const C : usize> Kinematic<C> for SerialKinematic<C> {
    // Segments
        fn segments(&self) -> &[KinElement; C] {
            &self.segments
        }

        fn segments_mut<'a>(&'a mut self) -> &'a mut [KinElement; C] {
            &mut self.segments
        }
    // 

    // TCP
        fn tcp<'a>(&'a self) -> &PointRef {
            &self.tcp
        }

        fn tcp_mut<'a>(&'a mut self) -> &'a mut PointRef {
            &mut self.tcp
        }
    // 

    fn calculate_end(&self) -> Position {
        let segments = self.segments(); 
        let mut pos_0 = Position::from(self.tcp().borrow().pos().clone());

        for i in 1 ..= C {
            let index = C - i;
            let point = segments[index].point().borrow();
    
            pos_0.transform(*point.ori());
            pos_0.shift(*point.pos());
        }

        pos_0
    }
}