use syact::prelude::*;
use sybot::prelude::*;

use syact::meas::take_simple_meas;
use sybot::robs::stepper::{LinearXYStepperRobot, LinearXYStepperActuators};


// pub constants
    pub const OFFSET_X : Delta = Delta(-50.0);
    pub const OFFSET_Y : Delta = Delta(-50.0);

    pub const PIN_STEP_X : u8 = 14;
    pub const PIN_STEP_Y : u8 = 15;
    pub const PIN_STEP_Z : u8 = 0;

    pub const PIN_DIR_X : u8 = 24;
    pub const PIN_DIR_Y : u8 = 25; 
    pub const PIN_DIR_Z : u8 = 0;

    pub const PIN_MEAS_X : u8 = 4;
    pub const PIN_MEAS_Y : u8 = 17;

    pub const RATIO_X : f32 = 6.0;
    pub const RATIO_Y : f32 = 6.0;

    pub const MEAS_DATA_X : SimpleMeasData = SimpleMeasData {
        set_gamma: Gamma(0.0),
        max_dist: Delta(-300.0),
        meas_speed: unsafe { SpeedFactor::from_unchecked(0.4) },

        add_samples: 0,
        sample_dist: Some(Delta(20.0))
    };

    pub const MEAS_DATA_Y : SimpleMeasData = SimpleMeasData {
        set_gamma: Gamma(0.0),
        max_dist: Delta(-300.0),
        meas_speed: unsafe { SpeedFactor::from_unchecked(0.4) },

        add_samples: 0,
        sample_dist: Some(Delta(20.0))
    };

    // pub const LIMITS_MIN : [Gamma; 2] = [

    // ]; 

    // Positions
    pub const HOME : [Phi; 2] = [
        Phi(125.0),
        Phi(125.0)
    ];

    pub fn linear_xy_robot_new() -> LinearXYStepperRobot {
        LinearXYStepperRobot::new([
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
                        EndSwitch::new(false, Some(Direction::CCW), UniInPin::new(PIN_MEAS_X))
                            .setup_inline().unwrap()
                    ))
                , RATIO_X
            ),
            y: LinearAxis::new(
                Stepper::new(GenericPWM::new(PIN_STEP_Y, PIN_DIR_Y).unwrap(), StepperConst::MOT_17HE15_1504S)
                    .add_interruptor_inline(Box::new(
                        EndSwitch::new(false, Some(Direction::CCW), UniInPin::new(PIN_MEAS_Y))
                            .setup_inline().unwrap()
                    ))
                , RATIO_Y
            )
        }, Vec::new())
    }
// 

// Station
    pub struct LinearXYStation { }

    impl Station<LinearXYStepperActuators, dyn StepperActuator, 2> for LinearXYStation {
        type Robot = LinearXYStepperRobot;

        fn home(&mut self, rob : &mut Self::Robot) -> Result<(), sybot::Error> {
            dbg!(take_simple_meas(&mut rob.comps_mut().x, &MEAS_DATA_X, SpeedFactor::MAX)?);
            dbg!(take_simple_meas(&mut rob.comps_mut().y, &MEAS_DATA_Y, SpeedFactor::MAX)?);

            dbg!(rob.move_abs_j_sync(HOME, SpeedFactor::from(0.25))?);     // Changed speed factor: FIX #1

            Ok(())
        }
    }
// 