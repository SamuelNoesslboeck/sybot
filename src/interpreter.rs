use crate::SyArm;
use stepper_lib::gcode::*;

pub fn init_interpreter(syarm : SyArm) -> Interpreter<SyArm> {
    let g10 : GCodeFunc<SyArm> = |arm, _, args | { 
        arm.drive_base_rel(args[0].value);
        None
    };

    let g11 : GCodeFunc<SyArm> = |arm, _, args | { 
        arm.drive_a1_rel(args[0].value);
        None
    };

    let g12 : GCodeFunc<SyArm> = |arm, _, args | { 
        arm.drive_a2_rel(args[0].value);
        None
    };

    let g13 : GCodeFunc<SyArm> = |arm, _, args | { 
        arm.drive_a3_rel(args[0].value);
        None
    };

    let m0 : GCodeFunc<SyArm> = |arm, _, _| { 
        arm.debug_pins(); 
        None
    };

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