use crate::SyArm;
use stepper_lib::gcode::*;

// General Functions
    /// G0 X<Position> Y<Position> Z<Position>
    /// Rapid positioning
    pub fn g0(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        let angles = arm.get_with_fixed_dec_rel(
            get_arg_letter(args, 'X'), 
            get_arg_letter(args, 'Y'), 
            get_arg_letter(args, 'Z'), 
            None
        );
        arm.drive_to_angles(arm.gammas_for_phis(&angles));
        None
    }

    /// G10 C<Angle>
    /// Moves the base with a relative angle
    pub fn g10(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_base_rel(args[0].value);
        None
    }

    /// G11 A<Angle>
    /// Moves the first arm segment with a relative angle
    pub fn g11(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a1_rel(args[0].value);
        None
    }

    /// G12 A<Angle>
    /// Moves the second arm segment with a relative angle
    pub fn g12(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a2_rel(args[0].value);
        None
    }

    /// G13 A<Angle>
    /// Moves the third arm segment with a relative angle
    pub fn g13(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a3_rel(args[0].value);
        None
    }

    /// G20 C<Angle>
    /// Moves the base with an absolute angle
    pub fn g20(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_base_rel(args[0].value);
        None
    }

    /// G21 A<Angle>
    /// Moves the first arm segment with an absolute angle
    pub fn g21(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a1_rel(args[0].value);
        None
    }

    /// G22 A<Angle>
    /// Moves the second arm segment with an absolute angle
    pub fn g22(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a2_rel(args[0].value);
        None
    }

    /// G23 A<Angle>
    /// Moves the third arm segment with an absolute angle
    pub fn g23(arm : &mut SyArm, _code : &GCode, args : &Args) -> Option<()> {
        arm.drive_a3_rel(args[0].value);
        None
    }

    /// G28 \
    /// Return to home position
    pub fn g28(arm : &mut SyArm, _ : &GCode, _ : &Args) -> Option<()> {
        arm.measure(2);
        None
    }
//

// Misc Functions
    pub fn m0(arm : &mut SyArm, _ : &GCode, _ : &Args) -> Option<()> {
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
            (13, Command::new(g13, 1)),
            (28, Command::new(g28, 1))
        ])),
        (Letter::Miscellaneous, NumEntries::from([
            (0, Command::new(m0, 0))
        ]))
    ]);

    return Interpreter::new(syarm, funcs);
}