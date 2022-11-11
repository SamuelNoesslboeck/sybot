use crate::SyArm;
use stepper_lib::gcode::{GCode, GCodeFunc, NumEntries, Letter, LetterEntries, Interpreter};

// pub fn init_interpreter<'a>(syarm : &SyArm) -> Interpreter<'static, SyArm> {
//     let g0 : GCodeFunc<SyArm> = |intpr : &Interpreter<'a, SyArm>, code : &GCode, arm : &SyArm| { 
//         // syarm.debug_pins();
//         None
//     };

//     let funcs = LetterEntries::from([
//         (Letter::General, NumEntries::from([
//             (0, g0)
//         ])),
//         (Letter::Miscellaneous, NumEntries::from([

//         ]))
//     ]);

//     return Interpreter {
//         funcs,
//         mach: syarm
//     };
// }