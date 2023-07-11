use std::collections::HashMap;

use sybot_robs::{Robot, Descriptor};

use crate::Interpreter;

mod gfuncs;
pub use gfuncs::*;

pub type Letter = gcode::Mnemonic;
pub type GCode = gcode::GCode;
pub type Args = [gcode::Word];
pub type GCodeResult = Result<serde_json::Value, crate::Error>;

pub type GCodeFunc<R, D, S> = fn (&mut R, &mut D, &mut S, &GCode, &Args) -> GCodeResult;
pub type ToolChangeFunc<R, D, S> = fn (&mut R, &mut D, &mut S, usize) -> GCodeResult;

pub type NumEntries<R, D, S> = HashMap<u32, GCodeFunc<R, D, S>>;
pub type Entries<R, D, S> = HashMap<Letter, NumEntries<R, D, S>>;

pub type NotFoundFunc = fn (GCode) -> GCodeResult;

pub trait GCodeBasis<R, D, S> {
    fn add_functions(&mut self, funcs : &mut Entries<R, D, S>);
}

pub struct GCodeIntpr<R : Robot<C>, D : Descriptor<C>, S, const C : usize> {
    pub funcs : Entries<R, D, S>,
    pub tool_change : Option<ToolChangeFunc<R, D, S>>,

    not_found : NotFoundFunc
}

impl<R : Robot<C>, D : Descriptor<C>, S, const C : usize> GCodeIntpr<R, D, S, C> {   
    pub fn new(tool_change : Option<ToolChangeFunc<R, D, S>>, funcs : Entries<R, D, S>, not_found : NotFoundFunc) -> Self {
        return GCodeIntpr {
            funcs,
            tool_change,

            not_found            
        }
    }
}

impl<R : Robot<C>, D : Descriptor<C>, S, const C : usize> GCodeIntpr<R, D, S, C> {
    pub fn init() -> Self {
        let funcs = Entries::from([
            (Letter::General, NumEntries::from([
                (0, g0::<R, D, S, C> as GCodeFunc<R, D, S>),
                (4, g4::<R, D, S, C>),
                (28, g28::<R, D, S, C>),
                // (29, g29::<R, D, C>),
                (100, g100::<R, D, S, C>),
                (1000, g1000::<R, D, S, C>),
                (1100, g1100::<R, D, S, C>)
            ])), 
            (Letter::Miscellaneous, NumEntries::from([
                (3, m3::<R, D, S, C> as GCodeFunc<R, D, S>),
                (4, m4::<R, D, S, C>),
                (5, m5::<R, D, S, C>),
                (30, m30::<R, D, S, C>),
                (119, m119::<R, D, S, C>),
                (1006, m1006::<R, D, S, C>),
            ])), 
            (Letter::ProgramNumber, NumEntries::from([
                (0, o0::<R, D, S, C> as GCodeFunc<R, D, S>)
            ]))
        ]);
        
        GCodeIntpr::new(Some(t), funcs, |_| { 
            Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "Invalid GCode input")))
        })
    }
}

impl<R : Robot<C>, D : Descriptor<C>, S, const C : usize> Interpreter<R, D, S, GCodeResult, C> for GCodeIntpr<R, D, S, C> {
    fn interpret(&self, rob : &mut R, desc : &mut D, stat : &mut S, gc_str : &str) -> Vec<GCodeResult> {
        let mut res = vec![]; 

        let not_found = self.not_found;

        for gc_str_line in gc_str.split('\n') {
            for gc_line in gcode::parse(gc_str_line) {
                if gc_line.mnemonic() == Letter::ToolChange {
                    res.push(match self.tool_change {
                        Some(func) => func(rob, desc, stat, gc_line.major_number() as usize),
                        None => not_found(gc_line)
                    });

                    continue;
                }

                let func_res = get_func(&self.funcs, &gc_line);

                res.push(match func_res {
                    Some(func) => func(rob, desc, stat, &gc_line, gc_line.arguments()),
                    None => not_found(gc_line)
                });
            }
        }

        res
    }
}

// Parsing
    /// Get the GCode Function stored for the given code
    pub fn get_func<'a, R, D, S>(funcs : &'a Entries<R, D, S>, gc : &'a GCode) -> Option<&'a GCodeFunc<R, D, S>> {
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

    pub fn args_by_iterate(args : &Args, base_letter : char) -> Vec<f32> {
        let mut letters = Vec::new();
        
        loop {
            if let Some(arg) = arg_by_letter(args, (base_letter as u8 + letters.len() as u8) as char) {
                letters.push(arg);
            } else {
                break;
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