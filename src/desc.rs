use core::f32::consts::PI;

use glam::{Mat3, Vec3};
use stepper_lib::{units::*, SyncComp};
use sybot_pkg::{Package, SegmentInfo};
use sybot_rcs::{WorldObj, Point, Position};
use sybot_rcs::math::{full_atan, calc_triangle};
use sybot_robs::{AxisConf, SegmentChain, LinSegmentChain, StepperRobot, Segment, BasicRobot};

use crate::Descriptor;

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
    pub segments : LinSegmentChain<4>,

    wobj : WorldObj,
    conf : SyArmConf,
}

impl SyArmDesc {
    fn new(mut wobj : WorldObj, segments : &Vec<SegmentInfo>) -> Result<Self, crate::Error> {
        Ok(Self {
            segments: LinSegmentChain::from_wobj(segments, &mut wobj, "tcp")?,

            wobj,
            conf: SyArmConf::default()
        })
    }
}

impl SyArmDesc {
    pub fn base<'a>(&'a self) -> &'a Segment {
        &self.segments[0]
    }

    pub fn arm1<'a>(&'a self) -> &'a Segment {
        &self.segments[1]
    }

    pub fn arm2<'a>(&'a self) -> &'a Segment {
        &self.segments[2]
    }

    pub fn arm3<'a>(&'a self) -> &'a Segment {
        &self.segments[3]
    }
}

impl Descriptor<4> for SyArmDesc {
    // Axis config
        fn aconf<'a>(&'a self) -> &'a dyn AxisConf {
            &self.conf
        }

        fn aconf_mut<'a>(&'a mut self) -> &'a mut dyn AxisConf {
            &mut self.conf
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
        fn update(&mut self, _ : &mut dyn BasicRobot<4>, phis : &[Phi; 4]) -> Result<(), crate::Error> {
            self.segments.update(phis)?;
            Ok(())
        }
    // 

    // Calculate
        fn convert_pos(&self, rob : &mut dyn BasicRobot<4>, mut pos : Position) 
        -> Result<[Phi; 4], sybot_robs::Error> {
            let phi_b = full_atan(pos.x(), pos.y());
            let dec_ang = self.aconf().phis()[0].0;

            let z_matr = Mat3::from_rotation_z(phi_b);
            let mut tcp_vec = self.segments.tcp().pos();
            tcp_vec = z_matr * tcp_vec;
            tcp_vec = Mat3::from_rotation_y(-dec_ang) * tcp_vec;

            pos.shift(-tcp_vec);
            pos.shift(-*self.wobj.pos());
            pos.shift(-self.segments[0].pos()); 
            pos.transform(Mat3::from_rotation_z(-phi_b)); 
            pos.shift(-self.segments[1].pos());
 
            let arm2 = self.arm2().pos();
            let arm3 = self.arm3().pos();

            let (alpha_2, _, gamma_2) = 
                calc_triangle(arm2.length(), arm3.length(), pos.pos().length()); 

            let mut pos_ang = Vec3::X.angle_between(*pos.pos());

            if pos.z() < 0.0 {
                pos_ang = -pos_ang;
            }

            let phi_1 = alpha_2 + pos_ang;
            let phi_2 = gamma_2 - PI;
            let phis = [ Phi(phi_b), Phi(phi_1), Phi(phi_2), Phi(dec_ang - phi_1 - phi_2) ];

            rob.valid_phis(&phis)?;

            Ok(phis) 
        }
    // 
}

pub struct SyArm {
    pub rob : StepperRobot<[Box<dyn SyncComp>; 4], 4>, 
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
