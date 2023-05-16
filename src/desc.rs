use stepper_lib::{units::*, Setup};
use sybot_robs::AxisConf;

use crate::RobotDesc;

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
    pub conf : Box<dyn AxisConf>
}

impl RobotDesc<4> for SyArmDesc {
    fn apply_aconf(&mut self, conf : Box<dyn AxisConf>) -> Result<(), sybot_robs::Error> {
        self.conf = conf;
        Ok(())
    }

    fn aconf<'a>(&'a self) -> &'a Box<dyn AxisConf> {
        &self.conf
    }

    // Event
        fn setup(&mut self, rob : &mut dyn sybot_robs::ComplexRobot<4>) -> Result<(), sybot_robs::Error> {
            Ok(())
        }

        fn update(&mut self, _ : &mut dyn sybot_robs::ComplexRobot<4>, _ : &[Phi; 4]) -> Result<(), sybot_robs::Error> {
            Ok(())
        }
    // 

    // Calculate
        fn convert_pos(&self, _ : sybot_rcs::Position) -> Result<[Phi; 4], sybot_robs::Error> {
            
        }
    //
}