use crate::SyArm;
use stepper_lib::gcode::{GCode, GCodeFunc, NumEntries, Letter, LetterEntries, Interpreter};

pub fn init_interpreter(syarm : SyArm) -> (SyArm, Interpreter) {
    let g0 : GCodeFunc = |intpr : &Interpreter, code : &GCode| { 
        // syarm.debug_pins();
        None
    };

    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, g0)
        ])),
        (Letter::Miscellaneous, NumEntries::from([

        ]))
    ]);

    return (syarm, Interpreter::new(funcs));
}