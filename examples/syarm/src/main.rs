use core::f32::consts::PI;
use std::time::Instant;

use glam::{Mat3, Vec3};
use rustyline::Editor;
use serde::{Deserialize, Serialize};

use syact::prelude::*;
use sybot::prelude::*;

// Robot
    #[derive(SyncCompGroup, Deserialize, Serialize, StepperCompGroup)]
    struct SyArmComps {
        base : GearJoint<Stepper>,
        arm1 : CylinderTriangle<Stepper>,
        arm2 : CylinderTriangle<Stepper>,
        arm3 : GearJoint<Stepper>
    }

    type SyArmRob = StepperRobot<SyArmComps, 4>;
// 

// Descriptor
    #[derive(Default, Debug)]
    pub struct SyArmConf {
        pub phis : [Phi; 1]
    }

    impl AxisConf for SyArmConf {
        fn phis<'a>(&'a self) -> &'a [Phi] {
            &self.phis
        }

        fn configure(&mut self, phis : Vec<Phi>) -> Result<(), sybot::Error> {
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
        fn new(mut wobj : WorldObj, segments : &Vec<SegmentInfo>) -> Result<Self, sybot::Error> {
            let tcp = PointRef::new(Position::new(Vec3::ZERO));
            wobj.add_point("tcp", tcp.clone());

            Ok(Self {
                segments: LinSegmentChain::from_wobj(segments, &mut wobj, "tcp")?,

                tcp,
                wobj,
                conf: SyArmConf::default()
            })
        }

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

    impl TryFrom<DescPackage> for SyArmDesc {
        type Error = sybot::Error;

        fn try_from(value: DescPackage) -> Result<Self, Self::Error> {
            Self::new(
                value.rcs.ok_or("A valid RCS must be provided for the robot!")?, 
                value.segments.as_ref().ok_or("A vaild set of segments must be provided for the robot!")?
            )
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

            fn current_tcp(&self) -> &PointRef {
                &self.tcp
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
            fn update(&mut self, _ : &mut dyn Robot<4>, phis : &[Phi; 4]) -> Result<(), sybot::Error> {
                self.segments.update(phis)?;
                
                let tcp_new = self.segments.calculate_end();
                let mut tcp = self.tcp.borrow_mut();

                *(tcp.pos_mut()) = *tcp_new.pos();
                *(tcp.ori_mut()) = *tcp_new.ori();

                Ok(())
            }
        // 

        // Calculate
            fn convert_pos(&self, rob : &dyn Robot<4>, mut pos : Position) 
            -> Result<[Phi; 4], sybot::Error> {
                let phi_b = sybot::math::full_atan(pos.x(), pos.y());
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
                    sybot::math::calc_triangle(arm2.length(), arm3.length(), pos.pos().length()); 

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
// 

// Station
    struct SyArmStation { 

    }

    impl TryFrom<StationPackage> for SyArmStation {
        type Error = sybot::Error;

        fn try_from(_: StationPackage) -> Result<Self, Self::Error> {
            Ok(Self { })
        }
    }
// 

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // let broker_addr = "syhub:1883".to_owned(); // std::env::var("SYARM_BROKER_ADDR").expect("SYARM_BROKER_ADDR must be set");

    println!("[SyArm ROS system] \nBasic robot operating system for the SyArm robot. (c) Samuel Noesslboeck 2023\n");
    println!("Initialising ... ");

    let pkg = Package::load(".")?;
    println!("- Loaded package: '{}'", pkg.info.name);

    let ( info, mut rob, mut desc, mut stat ) = pkg.unpack::<SyArmRob, SyArmDesc, SyArmStation>()?;

    dbg!(info);
    
    

    // Remotes and interpreters
    let gcode = sybot::gcode::GCodeIntpr::init();

    // let mqtt = Box::new(
    //    sybot::mqtt::Publisher::new(&broker_addr, "syarm-rob-client")?);

    // if mqtt.connect().is_ok() {
    //     println!("- Successfully connected to MQTT-broker ({})", broker_addr);
    //     rob.add_remote(mqtt);
    // } else {
    //     eprintln!("- Failed to connect to broker");
    // }

    // Setup
        rob.setup()?;
        println!("- Setup complete");

        print!("- Setting home position... ");
        rob.auto_meas()?;
        println!("done!");
    //

    let rob_phis = rob.phis();
    desc.update(&mut rob, &rob_phis)?;  

    rob.move_p_sync(&mut desc, Position::new(Vec3::new(330.0, 0.0, 400.0)), 1.0)?;

    let rob_phis = rob.phis();
    desc.update(&mut rob, &rob_phis)?;

    let inst = Instant::now();

    rob.move_l(&mut desc, Vec3::new(0.0, 0.0, 50.0), 0.5, Omega(50.0))?;

    println!("{:?}", inst.elapsed().as_secs_f32());

    rob.await_inactive()?;

    let rob_phis = rob.phis();
    desc.update(&mut rob, &rob_phis)?;
    println!("{:?}", desc.current_tcp().pos());

    println!("\nGCode interpreter");

    let mut editor = Editor::<(), _>::new().expect("Failed to make rustyline editor");

    loop {
        match editor.readline("> ") {
            Ok(input) => {
                editor.add_history_entry(&input)?;
                for res in gcode.interpret(&mut rob, &mut desc, &mut stat, &input) {
                    println!("{}\n", serde_json::to_string_pretty(&res?).unwrap());
                }
            },
            Err(_) => return Ok(()),
        }
    }
}
