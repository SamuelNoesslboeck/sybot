use std::{thread, time::Duration};

use serde_json::Value;
use stepper_lib::gcode::*;

use crate::{SyArm, SyArmResult, Robot};

// General Functions
    /// G0 X{Position} Y{Position} Z{Position} D{Angle} \
    /// Rapid positioning
    pub fn g0(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        let angles = arm.get_with_fixed_dec_s(
            get_arg_letter(args, 'X'), 
            get_arg_letter(args, 'Y'), 
            get_arg_letter(args, 'Z'), 
            get_arg_letter(args, 'D')
        )?; 
        arm.drive_abs(arm.gammas_from_phis(angles));
        arm.update_sim();
        Ok(serde_json::json!(angles))
    }

    /// G4 X{Seconds} P{Milliseconds}
    /// Dwell (sleeping)
    pub fn g4(_ : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        let seconds = 
            get_arg_letter(args, 'X').unwrap_or(0.0)            // Seconds
            + get_arg_letter(args, 'P').unwrap_or(0.0)/1000.0;  // Milliseconds
        thread::sleep(Duration::from_secs_f32(seconds));
        Ok(serde_json::json!(seconds))
    }

    /// G8 X{Position} Y{Position} Z{Position} D{Angle} \
    /// Rapid positioning async
    pub fn g8(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        let angles = arm.get_with_fixed_dec_s(
            get_arg_letter(args, 'X'), 
            get_arg_letter(args, 'Y'), 
            get_arg_letter(args, 'Z'), 
            get_arg_letter(args, 'D')
        )?; 
        arm.drive_abs_async(arm.gammas_from_phis(angles));
        arm.await_inactive();
        arm.update_sim();
        Ok(serde_json::json!(angles))
    }   

    // Move single axis
        // /// G10 C<Angle>
        // /// Moves the base with a relative angle
        // pub fn g10(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        //     arm.drive_base_rel(args[0].value);
        //     Ok(Value::Null)
        // }

        // /// G11 A<Angle>
        // /// Moves the first arm segment with a relative angle
        // pub fn g11(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        //     arm.drive_a1_rel(args[0].value);
        //     Ok(Value::Null)
        // }

        // /// G12 A<Angle>
        // /// Moves the second arm segment with a relative angle
        // pub fn g12(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        //     arm.drive_a2_rel(args[0].value);
        //     Ok(Value::Null)
        // }

        // /// G13 A<Angle>
        // /// Moves the third arm segment with a relative angle
        // pub fn g13(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        //     arm.drive_a3_rel(args[0].value);
        //     Ok(Value::Null)
        // }

        // /// G20 C<Angle>
        // /// Moves the base with an absolute angle
        // pub fn g20(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        //     arm.drive_base_rel(args[0].value);
        //     Ok(Value::Null)
        // }

        // /// G21 A<Angle>
        // /// Moves the first arm segment with an absolute angle
        // pub fn g21(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        //     arm.drive_a1_rel(args[0].value);
        //     Ok(Value::Null)
        // }

        // /// G22 A<Angle>
        // /// Moves the second arm segment with an absolute angle
        // pub fn g22(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        //     arm.drive_a2_rel(args[0].value);
        //     Ok(Value::Null)
        // }

        // /// G23 A<Angle>
        // /// Moves the third arm segment with an absolute angle
        // pub fn g23(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        //     arm.drive_a3_rel(args[0].value);
        //     Ok(Value::Null)
        // }
    // 

    /// G28 \
    /// Return to home position
    pub fn g28(arm : &mut SyArm, _ : &GCode, _ : &Args) -> SyArmResult<Value> {
        // arm.measure(2);
        match arm.measure(2) {
            Ok(_) => { },
            Err(meas) => println!(" -> Problems with measurement! {:?}", meas)
        };
        Ok(Value::Null)
    }

    /// G29 \
    /// Return to home position async
    pub fn g29(arm : &mut SyArm, _ : &GCode, _ : &Args) -> SyArmResult<Value> {
        // arm.measure(2);
        arm.measure_async(2);
        arm.await_inactive();
        arm.update_sim();
        arm.set_endpoint(arm.home_pos());
        Ok(Value::Null)
    }
//

// Misc Functions
    pub fn m0(_ : &mut SyArm, _ : &GCode, _ : &Args) -> SyArmResult<Value> {
        // arm.debug_pins();
        Ok(Value::Null)
    }

    pub fn m1(arm : &mut SyArm, _ : &GCode, _ : &Args) -> SyArmResult<Value> {
        println!("{}", arm.points_by_phis(&arm.all_phis())[3]);
        Ok(Value::Null)
    }
// 

// Tool change
//


pub fn init_interpreter(syarm : SyArm) -> Interpreter<SyArm, SyArmResult<Value>> {
    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, g0 as GCodeFunc<SyArm, SyArmResult<Value>>),
            (4, g4),
            (8, g8),
            // (10, g10),
            // (11, g11),
            // (12, g12),
            // (13, g13),
            (28, g28),
            (29, g29)
        ])),
        (Letter::Miscellaneous, NumEntries::from([
            (0, m0 as GCodeFunc<SyArm, SyArmResult<Value>>),
            (1, m1)
        ]))
    ]);

    Interpreter::new(syarm, funcs)
}


// New interpreter
mod funcs 
{
    use super::*;

    /// G0 X{Position} Y{Position} Z{Position} D{Angle} \
    /// Rapid positioning
    pub fn g0<R : Robot<N>, const N : usize>(arm : &mut R, _code : &GCode, args : &Args) -> Result<serde_json::Value, R::Error> {
        let angles = arm.get_with_fixed_dec_s(
            get_arg_letter(args, 'X'), 
            get_arg_letter(args, 'Y'), 
            get_arg_letter(args, 'Z'), 
            get_arg_letter(args, 'D')
        )?; 
        arm.drive_abs(arm.gammas_from_phis(angles));
        arm.update(&angles);
        
        Ok(serde_json::json!(angles))
    }
}

pub fn init_intpr<R : Robot<N>, const N : usize>(rob : R) -> Interpreter<R, Result<serde_json::Value, R::Error>> {
    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, funcs::g0::<R, N> as GCodeFunc<R, Result<serde_json::Value, R::Error>>)
        ]))
    ]);

    Interpreter::new(rob, funcs)
}