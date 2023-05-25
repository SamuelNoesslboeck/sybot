use glam::Mat3;
use stepper_lib::units::*;
use sybot_pkg::{Package, SegmentInfo};
use sybot_rcs::{WorldObj, PointRef, Point};
use sybot_rcs::math::full_atan;
use sybot_robs::{AxisConf, SegmentChain, LinSegmentChain, StepperRobot};

use crate::RobotDesc;

#[derive(Default, Debug)]
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

#[derive(Debug)]
pub struct SyArmDesc {
    wobj : WorldObj,
    pub segments : LinSegmentChain<4>,
    conf : Box<dyn AxisConf>
}

impl SyArmDesc {
    fn new(mut wobj : WorldObj, segments : &Vec<SegmentInfo>) -> Result<Self, crate::Error> {
        Ok(Self {
            segments: LinSegmentChain::from_wobj(segments, &mut wobj, "tcp")?,
            wobj,
            conf: Box::new(SyArmConf::default())
        })
    }
}

impl RobotDesc<4> for SyArmDesc {
    // Axis config
        fn apply_aconf(&mut self, conf : Box<dyn AxisConf>) -> Result<(), sybot_robs::Error> {
            if conf.phis().len() < 1 {
                Err(format!("The configuration requires more phis! (Required: {}, Given: {})", 1, conf.phis().len()).into())
            } else {
                self.conf = conf;
                Ok(())   
            }
        }

        fn aconf<'a>(&'a self) -> &'a Box<dyn AxisConf> {
            &self.conf
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

        fn update(&mut self, _ : &mut dyn sybot_robs::ComplexRobot<4>, phis : &[Phi; 4]) -> Result<(), sybot_robs::Error> {
            self.segments.update(phis)?;
            Ok(())
        }
    // 

    // Calculate
        fn convert_pos(&self, rob : &mut dyn sybot_robs::ComplexRobot<4>, mut pos : sybot_rcs::Position) 
        -> Result<[Phi; 4], sybot_robs::Error> {
            let phi_b = full_atan(pos.x(), pos.y());
            let phi_dec = self.aconf().phis()[0];

            let mut tcp_vec = *self.segments.tcp().borrow().pos();
            tcp_vec = Mat3::from_rotation_z(phi_b) * tcp_vec;
            tcp_vec = Mat3::from_rotation_y(-phi_dec) * tcp_vec;

            pos.shift(-tcp_vec);
            pos.shift(-self.wobj.pos());
            pos.shift(-(Mat3::from_rotation_z(phi_b) * self.segments[0].point));
        }
    // 
}

#[derive(Debug)]
pub struct SyArm {
    pub rob : StepperRobot<4>, 
    pub desc : SyArmDesc
}

impl TryFrom<Package> for SyArm {
    type Error = crate::Error; 

    fn try_from(pkg: Package) -> Result<Self, Self::Error> {
        let rob = StepperRobot::try_from(&pkg)?;
        let desc = if let Some(wobj) = pkg.wobj {       // TODO: Improve
            if let Some(segments) = &pkg.segments {
                SyArmDesc::new(wobj, segments)
            } else {
                Err("Define segments for this descriptor!".into())
            }
        } else {
            Err("The SyArm descriptor requires a vaild world object".into())
        }?;

        Ok(Self { rob, desc })
    }
}