use crate::SyArm;
use stepper_lib::gcode::{GCode, GCodeFunc, NumEntries, Letter, LetterEntries, Interpreter};

pub fn init_interpreter(syarm : SyArm) -> Interpreter<SyArm> {
    let m0 : GCodeFunc<SyArm> = |arm : &mut SyArm, _ : &GCode| { 
        arm.debug_pins(); 
        None
    };

    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, m0)
        ])),
        (Letter::Miscellaneous, NumEntries::from([
            
        ]))
    ]);

    return Interpreter::new(syarm, funcs);
}