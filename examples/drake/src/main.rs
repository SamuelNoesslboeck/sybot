use glam::Vec3;
use stepper_lib::prelude::*;
use sybot_lib::prelude::*;

pub struct DrakeDesc {
    pub segments : LinSegmentChain<3>,
    
    wobj : WorldObj,
    conf : EmptyConf,
}

impl DrakeDesc {
    pub fn new(mut wobj : WorldObj, segments : &Vec<SegmentInfo>) -> Result<Self, sybot_lib::Error> {
        Ok(Self {
            segments: LinSegmentChain::from_wobj(segments, &mut wobj, "tcp")?,
            wobj,
            conf: EmptyConf::default()
        })
    }
}

impl RobotDesc<3> for DrakeDesc {
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
        fn update(&mut self, _ : &mut dyn BasicRobot<3>, phis : &[Phi; 3]) -> Result<(), sybot_lib::Error> {
            self.segments.update(phis)
        }
    //

    // Calculate
        fn convert_pos(&self, rob : &mut dyn BasicRobot<3>, pos : Position) -> Result<[Phi; 3], sybot_lib::Error> {
            let phis = [ Phi(pos.x()), Phi(pos.y()), Phi(pos.z()) ];
            rob.valid_phis(&phis)?;
            Ok(phis)
        }
    // 
}

type DrakeRob = StepperRobot<[Cylinder; 3], 3>;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let pkg = Package::load("assets/DrakeRob")?;

    // Parse devices
        // Camera
        // Input panel
    // 
    
    // Parse robot
    let ( comps, _ ) = pkg.parse_comps_array()?;
    let mut rob = DrakeRob::from_pkg_stat(&pkg, comps)?;

    // Parse descriptor
    let (wobj, segments) = pkg.req_desc()?;
    let desc = DrakeDesc::new(wobj, &segments)?;

    // Actions
    rob.setup()?;
    rob.full_meas()?;

    // Loop
    let pos = Vec3::X * 100.0;
    let phis = desc.convert_pos(&mut rob, Position::new(pos))?;

    rob.move_abs_j_sync(phis, 1.0)?;
    // 

    Ok(())
}
