use syact::act::LinearAxis;
use syact::device::GenericPWM;
use syact::{units::*, Stepper, StepperConst};

use sybot::prelude::*;
use sybot::robs::stepper::{LinearXYZStepperRobot, LinearXYZStepperActuators};

const OFFSET_X : Delta = Delta(0.0);
const OFFSET_Y : Delta = Delta(0.0);
const OFFSET_Z : Delta = Delta(0.0);

const PIN_STEP_X : u8 = 0;
const PIN_STEP_Y : u8 = 0;
const PIN_STEP_Z : u8 = 0;

const PIN_DIR_X : u8 = 0;
const PIN_DIR_Y : u8 = 0; 
const PIN_DIR_Z : u8 = 0;

fn main() {
    let rob = LinearXYZStepperRobot::new([
        AngleConfig {
            offset: OFFSET_X,
            counter: false
        },
        AngleConfig {
            offset: OFFSET_Y,
            counter: false
        },
        AngleConfig {
            offset: OFFSET_Z,
            counter: false
        }
    ], LinearXYZStepperActuators {
        x: LinearAxis::new(
            Stepper::new(GenericPWM::new(PIN_STEP_X, PIN_DIR_X), StepperConst::MOT_17HE15_1504S)
        ),
        y: LinearAxis::new(
            Stepper::new(GenericPWM::new())
        )
    });
}