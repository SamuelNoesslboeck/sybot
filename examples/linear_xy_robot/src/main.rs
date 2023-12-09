use syact::meas::take_simple_meas;
use syact::prelude::*;
use sybot::desc::common::LinearXYDescriptor;
use sybot::prelude::*;

use sybot::robs::stepper::{LinearXYStepperRobot, LinearXYStepperActuators};

// Constants
    const OFFSET_X : Delta = Delta(0.0);
    const OFFSET_Y : Delta = Delta(0.0);

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
        max_dist: Delta(300.0),
        meas_speed_f: 0.2,

        add_samples: 2,
        sample_dist: Some(Delta(20.0))
    };

    const MEAS_DATA_Y : SimpleMeasData = SimpleMeasData {
        set_gamma: Gamma(0.0),
        max_dist: Delta(300.0),
        meas_speed_f: 0.2,

        add_samples: 2,
        sample_dist: Some(Delta(20.0))
    };

    // const LIMITS_MIN = [

    // ]; 

    // const
// 

// Station
    pub struct LinearXYStation { }

    impl Station<LinearXYStepperActuators, dyn StepperActuator, 2> for LinearXYStation {
        type Robot = LinearXYStepperRobot;

        fn home(&mut self, rob : &mut Self::Robot) -> Result<(), sybot::Error> {
            dbg!(take_simple_meas(&mut rob.comps_mut().x, &MEAS_DATA_X, 0.5)?);
            dbg!(take_simple_meas(&mut rob.comps_mut().y, &MEAS_DATA_Y, 0.5)?);
            Ok(())
        }
    }
// 

// Points
    #[derive(serde::Serialize, serde::Deserialize)]
    struct Line {
        p1 : [f32; 2],
        p2 : [f32; 2]
    }

    #[derive(serde::Serialize, serde::Deserialize)]
    struct PointsFile {
        contour : Vec<Line>
    }

    pub fn load_points(path : &str) -> PointsFile {
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
    }
// 

fn main() {
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
                    .add_interruptor_inline(Box::new(
                        EndSwitch::new(true, Some(Direction::CCW), UniInPin::new(PIN_MEAS_X))
                            .setup_inline().unwrap()
                    )),
                RATIO_X
            ),
            y: LinearAxis::new(
                Stepper::new(GenericPWM::new(PIN_STEP_Y, PIN_DIR_Y).unwrap(), StepperConst::MOT_17HE15_1504S)
                    .add_interruptor_inline(Box::new(
                        EndSwitch::new(true, Some(Direction::CCW), UniInPin::new(PIN_MEAS_Y))
                            .setup_inline().unwrap()
                    )),
                RATIO_Y
            )
        }, Vec::new());

        let mut desc = LinearXYDescriptor::new();

        let mut station = LinearXYStation { };
    // 

    rob.comps_mut().set_config(StepperConfig::new(12.0, 1.5));
    rob.setup().unwrap();

    station.home(&mut rob).unwrap();
}