use glam::Vec3;
use stepper_lib::prelude::*;
use sybot_lib::prelude::*;

pub struct DrakeDesc {
    pub segments : LinSegmentChain<3>,
    
    tcp : PointRef,
    wobj : WorldObj,
    conf : EmptyConf,
}

impl DrakeDesc {
    pub fn new(mut wobj : WorldObj, segments : &Vec<SegmentInfo>) -> Result<Self, sybot_lib::Error> {
        let tcp = PointRef::new(Position::new(Vec3::ZERO));
        wobj.add_point("tcp", tcp.clone());

        Ok(Self {
            segments: LinSegmentChain::from_wobj(segments, &mut wobj, "tcp")?,

            tcp,
            wobj,
            conf: EmptyConf::default()
        })
    }
}

impl Descriptor<3> for DrakeDesc {
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
        fn update(&mut self, _ : &mut dyn BasicRobot<3>, phis : &[Phi; 3]) -> Result<(), sybot_lib::Error> {
            self.segments.update(phis)?;

            let tcp_new = self.segments.calculate_end();
            let mut tcp = self.tcp.borrow_mut();

            *(tcp.pos_mut()) = *tcp_new.pos();
            *(tcp.ori_mut()) = *tcp_new.ori();

            Ok(())
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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let broker_addr = std::env::var("DRAKE_BROKER_ADDR").expect("DRAKE_BROKER_ADDR must be set");

    println!("Drake ROS system");
    println!("(c) Samuel Nösslböck 2023");

    let pkg = Package::load("assets/DrakeRob")?;
    println!(" => Loaded package: '{}'", pkg.info.name);

    // Parse devices
        // Camera
        

        // Input panel
        let start = Button::new(pkg.get_pin("input-panel/start").unwrap())?;
        // let off = InputDevice::new(pkg.get_pin("input-panel/off").unwrap())?;
    // 
    
    // Parse robot
    let ( comps, _ ) = pkg.parse_comps_array()?;
    let mut rob = DrakeRob::from_pkg_stat(&pkg, comps)?;

    // Parse descriptor
    let (wobj, segments) = pkg.req_desc()?;
    let mut desc = DrakeDesc::new(wobj, &segments)?;

    // Remotes and interpreters
    let gcode = sybot_lib::gcode::GCodeIntpr::init();

    let mqtt = Box::new(
        sybot_lib::mqtt::Publisher::new(&broker_addr, "drake-rob-client")?);
    
    if let Err(err) = mqtt.connect() {
        println!("MQTT connection failed: {}", err);
    }

    if mqtt.is_connected() {
        println!(" => Successfully connected to MQTT-broker ({})", broker_addr);
        rob.add_remote(mqtt);
    } else {
        eprintln!(" => Failed to connect to broker");
    }

    // Actions
    rob.setup()?;
    println!(" => Setup complete");

    rob.move_home()?;
    println!(" => Home position set");

    let rob_phis = rob.phis();
    desc.update(&mut rob, &rob_phis)?;

    // Loop
    start.wait(true).await?;

    loop {
        let mut line_buff = String::new();
        std::io::stdin().read_line(&mut line_buff)?;

        for res in gcode.interpret(&mut rob, &mut desc, &line_buff) {
            println!("{}\n", res?);
        }
    }
}
