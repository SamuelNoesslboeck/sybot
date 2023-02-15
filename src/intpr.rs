use std::{thread, time::Duration};

use serde_json::Value;
use stepper_lib::gcode::*;

use crate::SafeRobot;

mod funcs 
{
    use super::*;

    /// G0 X{Position} Y{Position} Z{Position} D{Angle} \
    /// Rapid positioning
    pub fn g0<R : SafeRobot<N>, const N : usize>(robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> {
        let angles =  match robot.safe_phis_for_vec(robot.safe_pos(
                get_arg_letter(args, 'X'), 
                get_arg_letter(args, 'Y'), 
                get_arg_letter(args, 'Z')
            ),
            robot.safe_deco(get_arg_letter(args, 'D'))
        ) {
            Ok(ang) => ang,
            Err((_, err)) => return Err(err)
        }; 

        robot.drive_abs(robot.gammas_from_phis(angles));
        robot.update(None);
        Ok(serde_json::json!(Vec::from(angles)))
    }

    /// G4 X{Seconds} P{Milliseconds}
    /// Dwell (sleeping)
    pub fn g4<R : SafeRobot<N>, const N : usize>(_ : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> {
        let seconds = 
            get_arg_letter(args, 'X').unwrap_or(0.0)            // Seconds
            + get_arg_letter(args, 'P').unwrap_or(0.0)/1000.0;  // Milliseconds
        thread::sleep(Duration::from_secs_f32(seconds));
        Ok(serde_json::json!(seconds))
    }

    /// G8 X{Position} Y{Position} Z{Position} D{Angle} \
    /// Rapid positioning async
    pub fn g8<R : SafeRobot<N>, const N : usize>(robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> {
        let angles =  match robot.safe_phis_for_vec(robot.safe_pos(
                get_arg_letter(args, 'X'), 
                get_arg_letter(args, 'Y'), 
                get_arg_letter(args, 'Z')
            ),
            robot.safe_deco(get_arg_letter(args, 'D'))
        ) {
            Ok(ang) => ang,
            Err((_, err)) => return Err(err)
        }; 
        robot.drive_abs_async(robot.gammas_from_phis(angles));
        robot.await_inactive();
        robot.update(None);
        Ok(serde_json::json!(Vec::from(angles)))
    }   

        /// G28 \
    /// Return to home position
    pub fn g28<R : SafeRobot<N>, const N : usize>(robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> {
        // arm.measure(2);
        match robot.measure(2) {
            Ok(_) => { },
            Err(meas) => println!(" -> Problems with measurement! {:?}", meas)
        };
        Ok(Value::Null)
    }

    /// G29 \
    /// Return to home position async
    pub fn g29<R : SafeRobot<N>, const N : usize>(robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> {
        // arm.measure(2);
        robot.measure_async(2);
        robot.await_inactive();
        robot.update(None);
        let home = *robot.home_pos();
        robot.set_endpoint(&home);
        Ok(Value::Null)
    }

    // Misc Functions
        pub fn m0<R : SafeRobot<N>, const N : usize>(_ : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> {
            // arm.debug_pins();
            Ok(Value::Null)
        }

        pub fn m1<R : SafeRobot<N>, const N : usize>(robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> {
            println!("{}", robot.points_from_phis(&robot.all_phis())[3]);
            Ok(Value::Null)
        }
    // 
}

pub fn init_intpr<R : SafeRobot<N>, const N : usize>(rob : R) -> Interpreter<R, Result<serde_json::Value, R::Error>> {
    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, funcs::g0::<R, N> as GCodeFunc<R, Result<serde_json::Value, R::Error>>),
            (4, funcs::g4::<R, N> ),
            (8, funcs::g8::<R, N> ),
            (28, funcs::g28::<R, N> ),
            (29, funcs::g29::<R, N> )
        ])), 
        (Letter::Miscellaneous, NumEntries::from([
            (0, funcs::m0::<R, N> as GCodeFunc<R, Result<serde_json::Value, R::Error>>),
            (1, funcs::m1::<R, N> )
        ]))
    ]);

    Interpreter::new(rob, funcs)
}