use serde_json::Value;
use std::{thread, time::Duration};
use stepper_lib::gcode::*;
use crate::{SyArm, SyArmResult};

// General Functions
    /// G0 X<Position> Y<Position> Z<Position>
    /// Rapid positioning
    pub fn g0(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        let angles = arm.get_with_fixed_dec_s(
            get_arg_letter(args, 'X'), 
            get_arg_letter(args, 'Y'), 
            get_arg_letter(args, 'Z'), 
                None
        )?; 
        arm.drive_to_angles(arm.gammas_for_phis(&angles));
        arm.update_sim();
        Ok(serde_json::json!(vec![angles.0, angles.1, angles.2, angles.3]))
    }

    /// G4 X<Position> Y<Position> Z<Position>
    /// Dwell (sleeping)
    pub fn g4(_ : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        let seconds = 
            get_arg_letter(args, 'X').unwrap_or(0.0)            // Seconds
            + get_arg_letter(args, 'P').unwrap_or(0.0)/1000.0;  // Milliseconds
        thread::sleep(Duration::from_secs_f32(seconds));
        Ok(serde_json::json!(seconds))
    }

    /// G8 X<Position> Y<Position> Z<Position>
    /// Rapid positioning async
    pub fn g8(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        let angles = arm.get_with_fixed_dec_s(
            get_arg_letter(args, 'X'), 
            get_arg_letter(args, 'Y'), 
            get_arg_letter(args, 'Z'), 
                None
        )?; 
        arm.drive_to_angles_async(arm.gammas_for_phis(&angles));
        arm.await_inactive();
        arm.update_sim();
        Ok(serde_json::json!(vec![angles.0, angles.1, angles.2, angles.3]))
    }

    /// G10 C<Angle>
    /// Moves the base with a relative angle
    pub fn g10(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        arm.drive_base_rel(args[0].value);
        Ok(Value::Null)
    }

    /// G11 A<Angle>
    /// Moves the first arm segment with a relative angle
    pub fn g11(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        arm.drive_a1_rel(args[0].value);
        Ok(Value::Null)
    }

    /// G12 A<Angle>
    /// Moves the second arm segment with a relative angle
    pub fn g12(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        arm.drive_a2_rel(args[0].value);
        Ok(Value::Null)
    }

    /// G13 A<Angle>
    /// Moves the third arm segment with a relative angle
    pub fn g13(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        arm.drive_a3_rel(args[0].value);
        Ok(Value::Null)
    }

    /// G20 C<Angle>
    /// Moves the base with an absolute angle
    pub fn g20(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        arm.drive_base_rel(args[0].value);
        Ok(Value::Null)
    }

    /// G21 A<Angle>
    /// Moves the first arm segment with an absolute angle
    pub fn g21(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        arm.drive_a1_rel(args[0].value);
        Ok(Value::Null)
    }

    /// G22 A<Angle>
    /// Moves the second arm segment with an absolute angle
    pub fn g22(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        arm.drive_a2_rel(args[0].value);
        Ok(Value::Null)
    }

    /// G23 A<Angle>
    /// Moves the third arm segment with an absolute angle
    pub fn g23(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<Value> {
        arm.drive_a3_rel(args[0].value);
        Ok(Value::Null)
    }

    /// G28 \
    /// Return to home position
    pub fn g28(arm : &mut SyArm, _ : &GCode, _ : &Args) -> SyArmResult<Value> {
        // arm.measure(2);
        arm.measure(2).unwrap();
        Ok(Value::Null)
    }
//

// Misc Functions
    pub fn m0(arm : &mut SyArm, _ : &GCode, _ : &Args) -> SyArmResult<Value> {
        arm.debug_pins();
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
            (10, g10),
            (11, g11),
            (12, g12),
            (13, g13),
            (28, g28)
        ])),
        (Letter::Miscellaneous, NumEntries::from([
            (0, m0 as GCodeFunc<SyArm, SyArmResult<Value>>)
        ]))
    ]);

    return Interpreter::new(syarm, funcs);
}