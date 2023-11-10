use syact::{SyncCompGroup, SyncComp};

use crate::{Robot, Descriptor};

// Submodules
    #[cfg(feature = "gcode")]
    pub mod gcode;

    #[cfg(feature = "lua")]
    pub mod lua; 
// 

/// Interpreters convert a string prompt into actions for the robot
pub trait Interpreter<G : SyncCompGroup<T, C>, R : Robot<G, T, C>, D : Descriptor<G, T, C>, S, T, O, const C : usize> 
where
    T : SyncComp + ?Sized + 'static
{
    /// Interpret a code string for a given robot
    fn interpret(&self, rob : &mut R, desc : &mut D, stat : &mut S, code : &str) -> Vec<O>; 

    /// Interpret a file for a given robot
    fn interpret_file(&self, rob : &mut R, desc : &mut D, stat : &mut S, path : &str) -> Vec<O> {
        self.interpret(rob, desc, stat, std::fs::read_to_string(path).unwrap().as_str())
    }
}