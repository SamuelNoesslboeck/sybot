use std::collections::HashMap;

use crate::Interpreter;
use crate::robot::SafeRobot;

use super::gfuncs;

pub type GCodeFunc<T, E> = fn (&mut T, &GCode, &Args) -> E;
pub type ToolChangeFunc<T, E> = fn (&mut T, usize) -> E;

pub type Letter = gcode::Mnemonic;
pub type GCode = gcode::GCode;
pub type Args = [gcode::Word];

pub type NumEntries<T, R> = HashMap<u32, GCodeFunc<T, R>>;
pub type LetterEntries<T, R> = HashMap<Letter, NumEntries<T, R>>;

pub type NotFoundFunc<R> = fn (GCode) -> R;

pub struct GCodeIntpr<Rob, Res>
{
    pub funcs : LetterEntries<Rob, Res>,
    pub tool_change : Option<ToolChangeFunc<Rob, Res>>,

    not_found : NotFoundFunc<Res>
}

impl<ROB, RES> GCodeIntpr<ROB, RES>
{   
    pub fn new(tool_change : Option<ToolChangeFunc<ROB, RES>>, funcs : LetterEntries<ROB, RES>, not_found : NotFoundFunc<RES>) -> Self {
        return GCodeIntpr {
            funcs,
            tool_change,

            not_found            
        }
    }
}

impl<ROB, RES> Interpreter<ROB, RES> for GCodeIntpr<ROB, RES> {
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

pub fn init_intpr<R : SafeRobot<COMP, DECO, DIM, ROT, Error = stepper_lib::Error>, 
    const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>() 
        -> GCodeIntpr<R, Result<serde_json::Value, R::Error>> 
{
    let funcs = LetterEntries::from([
        (Letter::General, NumEntries::from([
            (0, gfuncs::g0::<R, COMP, DECO, DIM, ROT> as GCodeFunc<R, Result<serde_json::Value, R::Error>>),
            (4, gfuncs::g4::<R, COMP, DECO, DIM, ROT>),
            (8, gfuncs::g8::<R, COMP, DECO, DIM, ROT>),
            (28, gfuncs::g28::<R, COMP, DECO, DIM, ROT>),
            // (29, gfuncs::g29::<R, COMP, DECO, DIM, ROT>),
            (100, gfuncs::g100::<R, COMP, DECO, DIM, ROT>),
            (1000, gfuncs::g1000::<R, COMP, DECO, DIM, ROT>),
            (1100, gfuncs::g1100::<R, COMP, DECO, DIM, ROT>)
        ])), 
        (Letter::Miscellaneous, NumEntries::from([
            (3, gfuncs::m3::<R, COMP, DECO, DIM, ROT> as GCodeFunc<R, Result<serde_json::Value, R::Error>>),
            (4, gfuncs::m4::<R, COMP, DECO, DIM, ROT>),
            (5, gfuncs::m5::<R, COMP, DECO, DIM, ROT>),
            (30, gfuncs::m30::<R, COMP, DECO, DIM, ROT>),
            (119, gfuncs::m119::<R, COMP, DECO, DIM, ROT>),
            (1006, gfuncs::m1006::<R, COMP, DECO, DIM, ROT>),
        ])), 
        (Letter::ProgramNumber, NumEntries::from([
            (0, gfuncs::o0::<R, COMP, DECO, DIM, ROT> as GCodeFunc<R, Result<serde_json::Value, R::Error>>)
        ]))
    ]);
    
    GCodeIntpr::new(Some(gfuncs::t), funcs, |_| { 
        Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, "Invalid GCode input")) 
    })
}