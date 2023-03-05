use std::{thread, time::Duration};

use serde_json::Value;
use stepper_lib::gcode::*;

use crate::SafeRobot;

mod gfuncs 
{
    use stepper_lib::Gamma;

    use super::*;

    // General functions
        /// G0 X{Position} Y{Position} Z{Position} DECO{Angle} \
        /// Rapid positioning
        pub fn g0<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> 
        {
            let pos = robot.safe_pos(
                arg_by_letter(args, 'X'), 
                arg_by_letter(args, 'Y'), 
                arg_by_letter(args, 'Z')
            );

            let phis =  match robot.safe_phis_for_vec(pos,
                robot.safe_deco(args_by_letter_fixed(args, 'D'))
            ) {
                Ok(ang) => ang,
                Err((_, err)) => return Err(err)
            }; 

            let deltas = robot.drive_abs(robot.gammas_from_phis(phis));
            robot.update(None);
            Ok(serde_json::json!({ 
                "points": pos.to_array(), 
                "phis": Vec::from(phis),
                "deltas": Vec::from(deltas)
            }))
        }

        /// G4 X{Seconds} P{Milliseconds}
        /// Dwell (sleeping)
        pub fn g4<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (_ : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> 
        {
            let seconds = 
                arg_by_letter(args, 'X').unwrap_or(0.0)            // Seconds
                + arg_by_letter(args, 'P').unwrap_or(0.0)/1000.0;  // Milliseconds
            thread::sleep(Duration::from_secs_f32(seconds));
            Ok(serde_json::json!(seconds))
        }

        /// G8 X{Position} Y{Position} Z{Position} DECO{Angle} \
        /// Rapid positioning async
        pub fn g8<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> 
        {
            let angles =  match robot.safe_phis_for_vec(robot.safe_pos(
                    arg_by_letter(args, 'X'), 
                    arg_by_letter(args, 'Y'), 
                    arg_by_letter(args, 'Z')
                ),
                [ 0.0; DECO ]
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
        pub fn g28<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error>  
        {
            // arm.measure(2);
            match robot.measure(2) {
                Ok(_) => { },
                Err(meas) => println!(" -> Problems with measurement! {:?}", meas)
            };
            Ok(Value::Null)
        }

        /// G29 \
        /// Return to home position async
        pub fn g29<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> 
        {
            // arm.measure(2);
            robot.measure_async(2);
            robot.await_inactive();
            robot.update(None);
            let home = *robot.home_pos();
            robot.set_endpoint(&home);
            Ok(Value::Null)
        }

        // Extra functions
        pub fn g100<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> 
        {
            let phis =  match robot.safe_phis(args_by_iterate_fixed::<COMP>(args, 'A')) {
                Ok(ang) => ang,
                Err((_, err)) => return Err(err)
            }; 

            let deltas = robot.drive_abs(robot.gammas_from_phis(phis));
            robot.update(None);
            Ok(serde_json::json!({ 
                "phis": Vec::from(phis),
                "deltas": Vec::from(deltas)
            }))
        }

        // Debug
        pub fn g1000<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> 
        {
            Ok(serde_json::json!({ 
                "phis": Vec::from(robot.all_phis()),
                "gammas": Vec::from(robot.all_gammas()),
                "pos": robot.pos().to_array()
            }))
        }

        pub fn g1100<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> 
        {
            let phis =  match robot.safe_phis(args_by_iterate_fixed::<COMP>(args, 'A')) {
                Ok(ang) => ang,
                Err((_, err)) => return Err(err)
            }; 

            robot.write_phis(&phis);
            robot.update(None);
            Ok(serde_json::json!(Vec::from(phis)))
        }
    //

    // Misc Functions
        pub fn m3<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> 
        {
            // TODO: Add response
            robot.activate_tool();
            robot.activate_spindle(true);

            Ok(Value::Null)
        }

        pub fn m4<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> 
        {
            Ok(serde_json::json!(robot.activate_spindle(false)))
        }

        pub fn m5<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> 
        {
            Ok(serde_json::json!(robot.deactivate_tool()))
        }

        // Additional functions
        pub fn m119<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> 
        {
            let gamma_opt = robot.gamma_tool();

            if let Some(gamma) = gamma_opt {
                return Ok(serde_json::json!(robot.rotate_tool_abs(Gamma(arg_by_letter(args, 'A').unwrap_or(gamma.0)))));
            }
            
            Ok(Value::Null)
        }

        // Debug functions
        pub fn m1006<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> 
        {
            Ok(serde_json::to_value(robot.get_tool().unwrap().get_json()).unwrap())
            // Ok(serde_json::to_value(
            //     robot.get_tools().iter().map(
            //         |t| t.get_json()
            //     ).collect::<Vec<serde_json::Value>>()
            // ).unwrap())
        }
    // 

    // Programm functions
        pub fn o0<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (_ : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, R::Error> 
        {
            println!("test");
            Ok(serde_json::Value::Null)
        }
    //

    // Tool
    pub fn t<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
        (robot : &mut R, index : usize) -> Result<serde_json::Value, R::Error> 
    {
        if let Some(tool) = robot.set_tool_id(index) {
            return Ok(tool.get_json())
        }

        Ok(serde_json::Value::Null)
        // Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, " -> No tool has been found for this index!"))
    }
}

pub fn init_intpr<R : SafeRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>(rob : R) -> Interpreter<R, Result<serde_json::Value, R::Error>> {
    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, gfuncs::g0::<R, COMP, DECO, DIM, ROT> as GCodeFunc<R, Result<serde_json::Value, R::Error>>),
            (4, gfuncs::g4::<R, COMP, DECO, DIM, ROT>),
            (8, gfuncs::g8::<R, COMP, DECO, DIM, ROT>),
            (28, gfuncs::g28::<R, COMP, DECO, DIM, ROT>),
            (29, gfuncs::g29::<R, COMP, DECO, DIM, ROT>),
            (100, gfuncs::g100::<R, COMP, DECO, DIM, ROT>),
            (1000, gfuncs::g1000::<R, COMP, DECO, DIM, ROT>),
            (1100, gfuncs::g1100::<R, COMP, DECO, DIM, ROT>)
        ])), 
        (Letter::Miscellaneous, NumEntries::from([
            (3, gfuncs::m3::<R, COMP, DECO, DIM, ROT> as GCodeFunc<R, Result<serde_json::Value, R::Error>>),
            (4, gfuncs::m4::<R, COMP, DECO, DIM, ROT>),
            (5, gfuncs::m5::<R, COMP, DECO, DIM, ROT>),
            (119, gfuncs::m119::<R, COMP, DECO, DIM, ROT>),
            (1006, gfuncs::m1006::<R, COMP, DECO, DIM, ROT>),
        ])), 
        (Letter::ProgramNumber, NumEntries::from([
            (0, gfuncs::o0::<R, COMP, DECO, DIM, ROT> as GCodeFunc<R, Result<serde_json::Value, R::Error>>)
        ]))
    ]);
    
    Interpreter::new(rob, Some(gfuncs::t), funcs)
}