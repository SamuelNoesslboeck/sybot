use std::time::Instant;

use syact::math::movements::DefinedActuator;
use syact::prelude::SyncActuatorGroup;
use syact::{StepperActuatorGroup, StepperConst};
use syact::act::{LinearAxis, StepperActuator};
use syact::act::stepper::{ComplexStepper, GenericPWM};
use syunit::*;
use tokio::task::JoinSet;

use crate::config::AngleConfig;
use crate::prelude::StepperRobot;
use crate::Robot;

// SimPin

// Helper structs
    pub struct SimPin {
        pub state : bool,
        pub state_changes : u64
    }

    impl SimPin {
        pub fn new() -> Self {
            Self {
                state: false,
                state_changes: 0
            }
        }
    }

    impl embedded_hal::digital::ErrorType for SimPin {
        type Error = embedded_hal::digital::ErrorKind;
    }

    impl embedded_hal::digital::OutputPin for SimPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.state = false;
            self.state_changes += 1;
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.state = true;
            self.state_changes += 1;
            Ok(())
        }
    }
// 

#[derive(StepperActuatorGroup)]
pub struct TestXYRobotComponents {
    pub x : LinearAxis<ComplexStepper<SimPin, SimPin>>,
    pub y : LinearAxis<ComplexStepper<SimPin, SimPin>>
}

impl TestXYRobotComponents {
    pub fn new() -> Self {
        Self {
            x: LinearAxis::new(
                ComplexStepper::new(GenericPWM::new(SimPin::new(), SimPin::new()).unwrap(), StepperConst::GEN).unwrap(),
                8.0
            ),
            y: LinearAxis::new(
                ComplexStepper::new(GenericPWM::new(SimPin::new(), SimPin::new()).unwrap(), StepperConst::GEN).unwrap(),
                8.0
            )
        }
    }
}

pub type TestXYRobot = StepperRobot<TestXYRobotComponents, dyn StepperActuator, 2>;

impl TestXYRobot {
    pub fn new_simple() -> Self {
        TestXYRobot::new([ AngleConfig::EMPTY; 2 ], TestXYRobotComponents::new(), vec![])
    }
}

#[tokio::test]
async fn move_j_test() {
    let mut rob = TestXYRobot::new_simple();

    const DELTAS : [Delta; 2] = [ Delta(100.0), Delta(200.0) ];
    const GEN_SPEED_F : Factor = Factor::MAX;

    println!("# sybot - move_j_test");
    println!("DELTAS: {:?}", DELTAS);
    println!("SPEED_F: {:?}", GEN_SPEED_F);

    // Code copied from move_j
    let gamma_0 = rob.gammas();
    let gamma_t = syunit::add_unit_arrays(gamma_0, DELTAS);
    let speed_f = syact::math::movements::ptp_speed_factors(
        rob.comps_mut(), gamma_0, gamma_t, GEN_SPEED_F
    );

    println!("> Calculated speed_f: {:?}", speed_f);
    println!("| > Time X: {} with {}", rob.comps().x.ptp_time_for_distance(gamma_0[0], gamma_t[0]), speed_f[0]);
    println!("| > Time >: {} with {}", rob.comps().y.ptp_time_for_distance(gamma_0[1], gamma_t[1]), speed_f[1]);

    let mut counter = 0;
    let inst = Instant::now();

    let mut set = JoinSet::new();

    for fut in rob.comps_mut().drive_rel(DELTAS, speed_f) {
        set.spawn(async move {
            fut.await.unwrap();
            counter
        });
        counter += 1;
    }

    while let Some(res) = set.join_next().await {
        println!("> Thread with id: {} compleded after: {} seconds", res.unwrap(), inst.elapsed().as_secs_f32());
    }
}