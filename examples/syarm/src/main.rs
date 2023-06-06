use core::f32::consts::PI;

use glam::{Mat3, Vec3};
use rustyline::Editor;
use serde::{Deserialize, Serialize};
use stepper_lib::prelude::*;
use sybot_lib::prelude::*;

#[derive(Default, Debug)]
pub struct SyArmConf {
    pub phis : [Phi; 1]
}

impl AxisConf for SyArmConf {
    fn phis<'a>(&'a self) -> &'a [Phi] {
        &self.phis
    }

    fn configure(&mut self, phis : Vec<Phi>) -> Result<(), sybot_lib::Error> {
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

    tcp : PointRef,
    wobj : WorldObj,
    conf : SyArmConf,
}

impl SyArmDesc {
    fn new(mut wobj : WorldObj, segments : &Vec<SegmentInfo>) -> Result<Self, sybot_lib::Error> {
        let tcp = PointRef::new(Position::new(Vec3::ZERO));
        wobj.add_point("tcp", tcp.clone());

        Ok(Self {
            segments: LinSegmentChain::from_wobj(segments, &mut wobj, "tcp")?,

            tcp,
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

        fn cache_tcp(&self, x_opt : Option<f32>, y_opt : Option<f32>, z_opt : Option<f32>) -> Vec3 {
            let pos = self.tcp.pos();

            Vec3::new(
                x_opt.unwrap_or(pos.x), 
                y_opt.unwrap_or(pos.y), 
                z_opt.unwrap_or(pos.z)
            )
        }
    // 

    // Events
        fn update(&mut self, _ : &mut dyn BasicRobot<4>, phis : &[Phi; 4]) -> Result<(), sybot_lib::Error> {
            self.segments.update(phis)?;
            
            let tcp_new = self.segments.calculate_end();
            let mut tcp = self.tcp.borrow_mut();

            *(tcp.pos_mut()) = *tcp_new.pos();
            *(tcp.ori_mut()) = *tcp_new.ori();

            Ok(())
        }
    // 

    // Calculate
        fn convert_pos(&self, rob : &mut dyn BasicRobot<4>, mut pos : Position) 
        -> Result<[Phi; 4], sybot_lib::Error> {
            let phi_b = sybot_lib::math::full_atan(pos.x(), pos.y());
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
                sybot_lib::math::calc_triangle(arm2.length(), arm3.length(), pos.pos().length()); 

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

#[derive(SyncCompGroup, Deserialize, Serialize)]
struct SyArmComps {
    base : GearJoint,
    arm1 : CylinderTriangle,
    arm2 : CylinderTriangle,
    arm3 : GearJoint
}

type SyArmRob = StepperRobot<SyArmComps, 4>;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let broker_addr = "syhub:1883"; // std::env::var("SYARM_BROKER_ADDR").expect("SYARM_BROKER_ADDR must be set");

    println!("[SyArm ROS system] \nBasic robot operating system for the SyArm robot. (c) Samuel Noesslboeck 2023\n");
    println!("Initialising ... ");

    let pkg = Package::load("assets/SyArm_Mk1")?;
    println!("- Loaded package: '{}'", pkg.info.name);
    
    // Parse robot
    let ( comps, _ ) = pkg.parse_comps_struct()?;
    let mut rob = SyArmRob::from_pkg_stat(&pkg, comps)?;

    // Parse descriptor
    let (wobj, segments) = pkg.req_desc()?;
    let mut desc = SyArmDesc::new(wobj, &segments)?;

    // Remotes and interpreters
    let gcode = sybot_lib::gcode::GCodeIntpr::init();

    let mqtt = Box::new(
        sybot_lib::mqtt::Publisher::new(broker_addr, "syarm-rob-client")?);

    if mqtt.connect().is_ok() {
        println!("- Successfully connected to MQTT-broker ({})", broker_addr);
        rob.add_remote(mqtt);
    } else {
        eprintln!("- Failed to connect to broker");
    }

    // Actions
    rob.setup()?;
    println!("- Setup complete");

    print!("- Setting home position... ");
    rob.move_home()?;
    println!("done!");

    let rob_phis = rob.phis();
    desc.update(&mut rob, &rob_phis)?;  

    println!("\nGCode interpreter");

    let mut editor = Editor::<(), _>::new().expect("Failed to make rustyline editor");

    loop {
        match editor.readline("> ") {
            Ok(input) => {
                editor.add_history_entry(&input)?;
                for res in gcode.interpret(&mut rob, &mut desc, &input) {
                    println!("{}\n", res?);
                }
            },
            Err(_) => return Ok(()),
        }
    }
}