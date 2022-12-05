use crate::SyArm;
use stepper_lib::gcode::*;

// General Functions
    /// G0 C<Angle>
    /// Moves the base with a relative angle
    // fn g0(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
    //     // Rapid movement
    //     None
    // }

    /// G10 C<Angle>
    /// Moves the base with a relative angle
    fn g10(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_base_rel(args[0].value);
        None
    }

    /// G11 A<Angle>
    /// Moves the first arm segment with a relative angle
    fn g11(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a1_rel(args[0].value);
        None
    }

    /// G12 A<Angle>
    /// Moves the second arm segment with a relative angle
    fn g12(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a2_rel(args[0].value);
        None
    }

    /// G13 A<Angle>
    /// Moves the third arm segment with a relative angle
    fn g13(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a3_rel(args[0].value);
        None
    }

    /// G20 C<Angle>
    /// Moves the base with an absolute angle
    fn g20(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_base_rel(args[0].value);
        None
    }

    /// G21 A<Angle>
    /// Moves the first arm segment with an absolute angle
    fn g21(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a1_rel(args[0].value);
        None
    }

    /// G22 A<Angle>
    /// Moves the second arm segment with an absolute angle
    fn g22(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a2_rel(args[0].value);
        None
    }

    /// G23 A<Angle>
    /// Moves the third arm segment with an absolute angle
    fn g23(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a3_rel(args[0].value);
        None
    }
//

// Misc Functions
    fn m0(arm : &mut SyArm, _ : &GCode, _ : &Args) -> Option<()> {
        arm.debug_pins();
        None
    }
// 

// Tool change
//


pub fn init_interpreter(syarm : SyArm) -> Interpreter<SyArm> {
    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (10, Command::new(g10, 1)),
            (11, Command::new(g11, 1)),
            (12, Command::new(g12, 1)),
            (13, Command::new(g13, 1))
        ])),
        (Letter::Miscellaneous, NumEntries::from([
            (0, Command::new(m0, 0))
        ]))
    ]);

    return Interpreter::new(syarm, funcs);
}