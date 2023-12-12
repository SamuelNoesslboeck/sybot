use indicatif::ProgressBar;
use syact::prelude::*;
use sybot::prelude::*;

use syact::meas::take_simple_meas;
use sybot::robs::stepper::{LinearXYStepperRobot, LinearXYStepperActuators};

// Constants
    const OFFSET_X : Delta = Delta(-80.0);
    const OFFSET_Y : Delta = Delta(-80.0);

    const PIN_STEP_X : u8 = 14;
    const PIN_STEP_Y : u8 = 15;

    const PIN_DIR_X : u8 = 24;
    const PIN_DIR_Y : u8 = 25; 

    const PIN_MEAS_X : u8 = 4;
    const PIN_MEAS_Y : u8 = 17;

    const RATIO_X : f32 = 6.0;
    const RATIO_Y : f32 = 6.0;

    const MEAS_DATA_X : SimpleMeasData = SimpleMeasData {
        set_gamma: Gamma(0.0),
        max_dist: Delta(-300.0),
        meas_speed_f: 0.4,

        add_samples: 0,
        sample_dist: Some(Delta(20.0))
    };

    const MEAS_DATA_Y : SimpleMeasData = SimpleMeasData {
        set_gamma: Gamma(0.0),
        max_dist: Delta(-300.0),
        meas_speed_f: 0.4,

        add_samples: 0,
        sample_dist: Some(Delta(20.0))
    };

    // const LIMITS_MIN = [

    // ]; 

    // const

    // Positions
    const HOME : [Phi; 2] = [
        Phi(150.0),
        Phi(150.0)
    ];
// 

// Station
    pub struct LinearXYStation { }

    impl Station<LinearXYStepperActuators, dyn StepperActuator, 2> for LinearXYStation {
        type Robot = LinearXYStepperRobot;

        fn home(&mut self, rob : &mut Self::Robot) -> Result<(), sybot::Error> {
            dbg!(take_simple_meas(&mut rob.comps_mut().x, &MEAS_DATA_X, 1.0)?);
            dbg!(take_simple_meas(&mut rob.comps_mut().y, &MEAS_DATA_Y, 1.0)?);

            dbg!(rob.move_abs_j_sync(HOME, 0.25)?);     // Changed speed factor: FIX #1

            Ok(())
        }
    }
// 

// Points
    const PIXEL_PER_MM : f32 = 4.0;

    #[derive(serde::Serialize, serde::Deserialize)]
    pub struct Line {
        p1 : [f32; 2],
        p2 : [f32; 2]
    }

    #[derive(serde::Serialize, serde::Deserialize)]
    pub struct LinesFile {
        contour : Vec<Line>
    }

    pub fn load_points(path : &str) -> LinesFile {
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
    }

    pub fn convert_pixel(pixel : f32) -> Phi {
        Phi(pixel / PIXEL_PER_MM)
    }

    pub fn convert_line(line : Line) -> [[Phi; 2]; 2] {
        [
            [ convert_pixel(line.p1[0]), convert_pixel(line.p1[1]) ],
            [ convert_pixel(line.p2[0]), convert_pixel(line.p2[1]) ]
        ]
    }
// 

fn main() {
    // Init logging
        env_logger::init();
    // 

    // RDS
        let mut rob = LinearXYStepperRobot::new([
            AngleConfig {
                offset: OFFSET_X,
                counter: false
            },
            AngleConfig {
                offset: OFFSET_Y,
                counter: false
            }
        ], LinearXYStepperActuators {
            x: LinearAxis::new(
                Stepper::new(GenericPWM::new(PIN_STEP_X, PIN_DIR_X).unwrap(), StepperConst::MOT_17HE15_1504S)
                    // .add_interruptor_inline(Box::new(
                    //     EndSwitch::new(false, Some(Direction::CCW), UniInPin::new(PIN_MEAS_X))
                    //         .setup_inline().unwrap()
                    // )),
                , RATIO_X
            ),
            y: LinearAxis::new(
                Stepper::new(GenericPWM::new(PIN_STEP_Y, PIN_DIR_Y).unwrap(), StepperConst::MOT_17HE15_1504S)
                    // .add_interruptor_inline(Box::new(
                    //     EndSwitch::new(false, Some(Direction::CCW), UniInPin::new(PIN_MEAS_Y))
                    //         .setup_inline().unwrap()
                    // )),
                , RATIO_Y
            )
        }, Vec::new());

        // let desc = LinearXYDescriptor::new();

        let mut station = LinearXYStation { };
    // 

    // Lines
        let lines = load_points("assets/sample_lines.json");
    // 

    rob.comps_mut().set_config(StepperConfig::new(12.0, 1.5));
    rob.setup().unwrap();

    println!("Driving to home position ... ");

    // station.home(&mut rob).unwrap();

    println!("Starting to draw ... ");

    let pb = ProgressBar::new(lines.contour.len() as u64);

    for line in lines.contour {
        let points = convert_line(line);

        log::debug!("Driving to {:?}", points[0]);
        // rob.move_abs_j(points[0], 0.25).unwrap();
        // rob.await_inactive().unwrap();
        rob.move_abs_j_sync(points[0], 0.25).unwrap();

        log::debug!("Driving to {:?}", points[1]);
        // rob.move_abs_j(points[1], 0.25).unwrap();
        // rob.await_inactive().unwrap();
        rob.move_abs_j_sync(points[1], 0.25).unwrap();

        pb.inc(1);
    }

    pb.finish_with_message("done");
    
    let mut buffer;
    loop {
        println!("X: ");

        buffer = String::new();
        std::io::stdin().read_line(&mut buffer).unwrap();

        let x = buffer.trim().parse::<Phi>().unwrap();

        println!("Y: ");

        buffer = String::new();
        std::io::stdin().read_line(&mut buffer).unwrap();

        let y = buffer.trim().parse::<Phi>().unwrap();

        rob.move_abs_j([ x, y ], 0.25).unwrap();
        rob.await_inactive().unwrap();
    }
}