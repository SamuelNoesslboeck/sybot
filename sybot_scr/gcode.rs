use std::collections::HashMap;

use crate::Interpreter;

mod gfuncs;
pub use gfuncs::*;
use sybot_robs::BasicRobot;

pub type GCodeFunc<T, D, E> = fn (&mut T, &mut D, &GCode, &Args) -> E;
pub type ToolChangeFunc<T, D, E> = fn (&mut T, &mut D, usize) -> E;

pub type Letter = gcode::Mnemonic;
pub type GCode = gcode::GCode;
pub type Args = [gcode::Word];

pub type NumEntries<T, D, R> = HashMap<u32, GCodeFunc<T, D, R>>;
pub type LetterEntries<T, D, R> = HashMap<Letter, NumEntries<T, D, R>>;

pub type NotFoundFunc<R> = fn (GCode) -> R;

pub struct GCodeIntpr<ROB, DESC, RES>
{
    pub funcs : LetterEntries<ROB, DESC, RES>,
    pub tool_change : Option<ToolChangeFunc<ROB, DESC, RES>>,

    not_found : NotFoundFunc<RES>
}

impl<ROB, DESC, RES> GCodeIntpr<ROB, DESC, RES>
{   
    pub fn new(tool_change : Option<ToolChangeFunc<ROB, DESC, RES>>, funcs : LetterEntries<ROB, DESC, RES>, not_found : NotFoundFunc<RES>) -> Self {
        return GCodeIntpr {
            funcs,
            tool_change,

            not_found            
        }
    }
}

impl<ROB, DESC, RES> Interpreter<ROB, DESC, RES> for GCodeIntpr<ROB, DESC, RES> {
    fn interpret(&self, rob : &mut ROB, gc_str : &str) -> Vec<RES> {
        let mut res = vec![]; 

        let not_found = self.not_found;

        for gc_str_line in gc_str.split('\n') {
            for gc_line in gcode::parse(gc_str_line) {
                if gc_line.mnemonic() == Letter::ToolChange {
                    res.push(match self.tool_change {
                        Some(func) => func(rob, gc_line.major_number() as usize),
                        None => not_found(gc_line)
                    });

                    continue;
                }

                let func_res = get_func(&self.funcs, &gc_line);

                res.push(match func_res {
                    Some(func) => func(rob, &gc_line, gc_line.arguments()),
                    None => not_found(gc_line)
                });
            }
        }

        res
    }
}

// Parsing
    /// Get the GCode Function stored for the given code
    pub fn get_func<'a, T, E>(funcs : &'a LetterEntries<T, E>, gc : &'a GCode) -> Option<&'a GCodeFunc<T, E>> {
        funcs.get(&gc.mnemonic()).and_then(|v| {
            v.get(&gc.major_number())
        })
    }

    // Argument parsing
    pub fn arg_by_letter(args : &Args, letter : char) -> Option<f32> {
        for i in 0 .. args.len() {
            if args[i].letter == letter {
                return Some(args[i].value);
            }
        }

        None
    }

    pub fn args_by_letter(args : &Args, letter : char) -> Vec<f32> {
        let mut letters = Vec::new();

        for i in 0 .. args.len() {
            if args[i].letter == letter {
                letters.push(args[i].value);
            }
        }

        letters
    }

    pub fn args_by_letter_fixed<const N : usize>(args : &Args, letter : char) -> [Option<f32>; N] {
        let mut letters = [None; N]; 
        let mut l_index : usize = 0;

        for i in 0 .. args.len() {
            if args[i].letter == letter {
                letters[l_index] = Some(args[i].value);
                l_index += 1;

                if l_index == N {
                    break;
                }
            }
        }   

        letters
    }

    pub fn args_by_iterate_fixed<const N : usize>(args : &Args, base_letter : char) -> [Option<f32>; N] {
        let mut letters = [None; N];
        
        for i in 0 .. N {
            letters[i] = arg_by_letter(args, (base_letter as u8 + i as u8) as char);
        }

        letters
    }
// 

pub fn init_intpr<R : BasicRobot<C>, const C : usize>() -> GCodeIntpr<R, Result<serde_json::Value, crate::Error>> {
    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, g0::<R, C> as GCodeFunc<R, Result<serde_json::Value, crate::Error>>),
            (4, g4::<R, C>),
            (8, g8::<R, C>),
            (28, g28::<R, C>),
            // (29, g29::<R, C>),
            (100, g100::<R, C>),
            (1000, g1000::<R, C>),
            (1100, g1100::<R, C>)
        ])), 
        (Letter::Miscellaneous, NumEntries::from([
            (3, m3::<R, C> as GCodeFunc<R, Result<serde_json::Value, crate::Error>>),
            (4, m4::<R, C>),
            (5, m5::<R, C>),
            (30, m30::<R, C>),
            (119, m119::<R, C>),
            (1006, m1006::<R, C>),
        ])), 
        (Letter::ProgramNumber, NumEntries::from([
            (0, o0::<R, C> as GCodeFunc<R, Result<serde_json::Value, crate::Error>>)
        ]))
    ]);
    
    GCodeIntpr::new(Some(t), funcs, |_| { 
        Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "Invalid GCode input")))
    })
}