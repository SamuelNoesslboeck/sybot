use crate::{SyArm, SyArmResult};
use stepper_lib::gcode::*;

// General Functions
    /// G0 X<Position> Y<Position> Z<Position>
    /// Rapid positioning
    pub fn g0(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        let angles = arm.get_with_fixed_dec_s(
            get_arg_letter(args, 'X'), 
            get_arg_letter(args, 'Y'), 
            get_arg_letter(args, 'Z'), 
                None
        )?;
        arm.drive_to_angles(arm.gammas_for_phis(&angles));
        arm.update_sim();
        Ok(())
    }

    /// G10 C<Angle>
    /// Moves the base with a relative angle
    pub fn g10(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        arm.drive_base_rel(args[0].value);
        Ok(())
    }

    /// G11 A<Angle>
    /// Moves the first arm segment with a relative angle
    pub fn g11(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        arm.drive_a1_rel(args[0].value);
        Ok(())
    }

    /// G12 A<Angle>
    /// Moves the second arm segment with a relative angle
    pub fn g12(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        arm.drive_a2_rel(args[0].value);
        Ok(())
    }

    /// G13 A<Angle>
    /// Moves the third arm segment with a relative angle
    pub fn g13(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        arm.drive_a3_rel(args[0].value);
        Ok(())
    }

    /// G20 C<Angle>
    /// Moves the base with an absolute angle
    pub fn g20(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        arm.drive_base_rel(args[0].value);
        Ok(())
    }

    /// G21 A<Angle>
    /// Moves the first arm segment with an absolute angle
    pub fn g21(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        arm.drive_a1_rel(args[0].value);
        Ok(())
    }

    /// G22 A<Angle>
    /// Moves the second arm segment with an absolute angle
    pub fn g22(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        arm.drive_a2_rel(args[0].value);
        Ok(())
    }

    /// G23 A<Angle>
    /// Moves the third arm segment with an absolute angle
    pub fn g23(arm : &mut SyArm, _code : &GCode, args : &Args) -> SyArmResult<()> {
        arm.drive_a3_rel(args[0].value);
        Ok(())
    }

    /// G28 \
    /// Return to home position
    pub fn g28(arm : &mut SyArm, _ : &GCode, _ : &Args) -> SyArmResult<()> {
        arm.measure(2);
        Ok(())
    }
//

// Misc Functions
    pub fn m0(arm : &mut SyArm, _ : &GCode, _ : &Args) -> SyArmResult<()> {
        arm.debug_pins();
        Ok(())
    }
// 

// Tool change
//


pub fn init_interpreter(syarm : SyArm) -> Interpreter<SyArm, SyArmResult<()>> {
    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, g0 as GCodeFunc<SyArm, SyArmResult<()>>),
            (10, g10),
            (11, g11),
            (12, g12),
            (13, g13),
            (28, g28)
        ])),
        (Letter::Miscellaneous, NumEntries::from([
            (0, m0 as GCodeFunc<SyArm, SyArmResult<()>>)
        ]))
    ]);

    return Interpreter::new(syarm, funcs);
}