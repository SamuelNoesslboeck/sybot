use serde::de::IntoDeserializer;
use stepper_lib::units::*;
use sybot_pkg::Package;
use sybot_rcs::{WorldObj, PointRef, Point};
use sybot_robs::{AxisConf, SegmentChain, LinSegmentChain};

use crate::RobotDesc;

#[derive(Default)]
pub struct SyArmConf {
    pub phis : [Phi; 1]
}

impl AxisConf for SyArmConf {
    fn phis<'a>(&'a self) -> &'a [Phi] {
        &self.phis
    }

    fn configure(&mut self, phis : Vec<Phi>) -> Result<(), crate::Error> {
        if phis.len() < 1 {
            Err("Not enough angles for configuring the axis configuration! (1 required)".into())
        } else {
            self.phis[0] = phis[0];
            Ok(())
        }
    }
}

pub struct SyArmDesc {
    wobj : WorldObj,
    segments : LinSegmentChain<4>,
    conf : Box<dyn AxisConf>
}

impl TryFrom<Package> for SyArmDesc {
    type Error = crate::Error;

    fn try_from(pkg: Package) -> Result<Self, Self::Error> {
        if let Some(mut wobj) = pkg.wobj {
            if let Some(segments) = &pkg.segments {
                Ok(Self {
                    segments: LinSegmentChain::from_wobj(segments, &mut wobj)?,
                    wobj,
                    conf: Box::new(SyArmConf::default())
                })
            } else {
                Err("Define segments for this descriptor!".into())
            }
        } else {
            Err("The SyArm descriptor requires a vaild world object".into())
        }
    }
}

impl RobotDesc<4> for SyArmDesc {
    // Axis config
        fn apply_aconf(&mut self, conf : Box<dyn AxisConf>) -> Result<(), sybot_robs::Error> {
            self.conf = conf;
            Ok(())
        }

        fn aconf<'a>(&'a self) -> &'a Box<dyn AxisConf> {
            &self.conf
        }
    //

    // Segments
        fn segments<'a>(&'a self) -> &'a dyn SegmentChain<4> {
            &self.segments
        }

        fn segments_mut<'a>(&'a mut self) -> &'a mut dyn SegmentChain<4> {
            &mut self.segments
        }
    // 

    // World object
        fn wobj<'a>(&'a self) -> &'a WorldObj {
            &self.wobj
        }

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj {
            &mut self.wobj
        }
    // 

    // Events
        fn setup(&mut self, _ : &mut dyn sybot_robs::ComplexRobot<4>) -> Result<(), sybot_robs::Error> {
            Ok(())
        }

        fn update(&mut self, _ : &mut dyn sybot_robs::ComplexRobot<4>, _ : &[Phi; 4]) -> Result<(), sybot_robs::Error> {
            Ok(())
        }
    // 

    // Calculate
        fn convert_pos(&self, rob : &mut dyn sybot_robs::ComplexRobot<4>, pos : sybot_rcs::Position) -> Result<[Phi; 4], sybot_robs::Error> {
            todo!()
        }
    // 
}