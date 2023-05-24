use stepper_lib::units::*;
use sybot_rcs::{Point, WorldObj};
use sybot_robs::{AxisConf, SegmentChain};

use crate::RobotDesc;

pub struct SyArmConf {
    pub phis : [Phi; 1],
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
    conf : Box<dyn AxisConf>,
    wobj : WorldObj,
    segments : Box<dyn SegmentChain<4>>
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

    // 

    // 

    // Event
        fn setup(&mut self, _ : &mut dyn sybot_robs::ComplexRobot<4>) -> Result<(), sybot_robs::Error> {
            Ok(())
        }

        fn update(&mut self, _ : &mut dyn sybot_robs::ComplexRobot<4>, _ : &[Phi; 4]) -> Result<(), sybot_robs::Error> {
            Ok(())
        }
    // 

    // Calculate
        fn convert_pos(&self, rob : &mut dyn sybot_robs::ComplexRobot<4>, pos : sybot_rcs::Position) -> Result<[Phi; 4], sybot_robs::Error> {
            
        }
    //
}