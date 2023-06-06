use core::ops::Index;

use glam::{Vec3, Mat3};
use stepper_lib::units::*;

use sybot_pkg::{SegmentInfo, SegmentMovementInfo};
use sybot_rcs::{PointRef, Position, Point, WorldObj};

#[derive(Debug)]
pub enum SegmentMovement {
    Rotation,
    Linear(Vec3)
}

impl From<SegmentMovementInfo> for SegmentMovement {
    fn from(info : SegmentMovementInfo) -> Self {
        match info {
            SegmentMovementInfo::Rotation => SegmentMovement::Rotation,
            SegmentMovementInfo::Linear(fvec) => SegmentMovement::Linear(Vec3::from(fvec))
        }
    }
}

#[derive(Debug)]
pub struct Segment {
    movement : SegmentMovement,
    point_0 : PointRef,

    phi : Phi,
    pub point : PointRef
}

impl Segment {
    pub fn new(point : PointRef, info : &SegmentInfo) -> Self {
        Self {
            phi: Phi::ZERO,
            movement: SegmentMovement::from(info.movement),

            point_0 : point.clone_no_ref(),
            point
        }
    }

    #[inline]
    pub fn pos(&self) -> Vec3 {
        self.point.pos()    
    }

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
    // Segments
        fn segments<'a>(&'a self) -> &'a [Segment; C]; 

        fn segments_mut<'a>(&'a mut self) -> &'a mut [Segment; C];

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
pub struct LinSegmentChain<const C : usize> {
    segments : [Segment; C],
    tcp : PointRef
}

impl<const C : usize> Index<usize> for LinSegmentChain<C> {
    type Output = Segment;

    fn index(&self, index: usize) -> &Self::Output {
        &self.segments[index]
    }
}

impl<const C : usize> LinSegmentChain<C> {
    pub fn from_wobj(infos : &Vec<SegmentInfo>, wobj : &mut WorldObj, tcp_name : &str) -> Result<Self, crate::Error> {
        if infos.len() < C {
            return Err(format!("Not enough segments declared! (Required: {}, Given: {})", C, infos.len()).into())
        }

        let mut path = vec![];
        let mut segments = vec![];

        for i in 0 .. C {
            path.push(infos[i].info.name.as_str());

            let seg = Segment::new(wobj.req_point_path(&path)?, &infos[i]);

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

impl<const C : usize> SegmentChain<C> for LinSegmentChain<C> {
    // Segments
        fn segments<'a>(&'a self) -> &'a [Segment; C] {
            &self.segments
        }

        fn segments_mut<'a>(&'a mut self) -> &'a mut [Segment; C] {
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