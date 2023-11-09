use core::ops::Index;

use syact::units::*;

use crate::rcs::{PointRef, Position};
use crate::robs::KinElement;

pub trait Kinematic<const C : usize> : core::fmt::Debug {
    // Segments
        fn segments<'a>(&'a self) -> &'a [KinElement; C]; 

        fn segments_mut<'a>(&'a mut self) -> &'a mut [KinElement; C];

        fn tcp<'a>(&'a self) -> &'a PointRef;

        fn tcp_mut<'a>(&'a mut self) -> &'a mut PointRef;
    // 

    // Data
        fn phis<'a>(&'a self) -> [Phi; C] {
            let mut phis = [Phi::ZERO; C];
            for i in 0 .. C {
                phis[i] = self.segments()[i].phi;
            }
            phis
        }
    // 
    
    fn calculate_end(&self) -> Position {
        let segments = self.segments(); 
        let mut pos_0 = Position::new(self.tcp().borrow().pos().clone());

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
pub struct SerialKinematic<const C : usize> {
    segments : [KinElement; C],
    tcp : PointRef
}

impl<const C : usize> Index<usize> for SerialKinematic<C> {
    type Output = KinElement;

    fn index(&self, index: usize) -> &Self::Output {
        &self.segments[index]
    }
}

impl<const C : usize> SerialKinematic<C> {
    pub fn from_wobj(infos : &Vec<SegmentInfo>, wobj : &mut WorldObj, tcp_name : &str) -> Result<Self, crate::Error> {
        if infos.len() < C {
            return Err(format!("Not enough segments declared! (Required: {}, Given: {})", C, infos.len()).into())
        }

        let mut path = vec![];
        let mut segments = vec![];

        for i in 0 .. C {
            path.push(infos[i].info.name.as_str());

            let seg = KinElement::new(wobj.req_point_path(&path)?, &infos[i]);

            let mut init_name = infos[i].info.name.clone();
            init_name.push_str("_init");

            wobj.add_point(init_name, seg.point_0.clone());
            segments.push(seg);
        }

        path.push(tcp_name);

        Ok(Self {
            segments: segments.try_into().unwrap(),
            tcp: wobj.req_point_path(&path)?
        })
    }

    pub fn start_point(&self) -> &PointRef {
        &self.segments[0].point
    }
}

impl<const C : usize> Kinematic<C> for SerialKinematic<C> {
    // Segments
        fn segments<'a>(&'a self) -> &'a [KinElement; C] {
            &self.segments
        }

        fn segments_mut<'a>(&'a mut self) -> &'a mut [KinElement; C] {
            &mut self.segments
        }
    // 

    // TCP
        fn tcp<'a>(&'a self) -> &'a PointRef {
            &self.tcp
        }

        fn tcp_mut<'a>(&'a mut self) -> &'a mut PointRef {
            &mut self.tcp
        }
    // 
}