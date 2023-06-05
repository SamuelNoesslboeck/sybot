extern crate alloc;

// Submodules
    #[cfg(feature = "gcode")]
    pub mod gcode;

    #[cfg(feature = "lua")]
    pub mod lua; 
// 

pub type Error = Box<dyn std::error::Error>;

/// Interpreters convert a string prompt into actions for the robot
pub trait Interpreter<T, D, R> {
    /// Interpret a code string for a given robot
    fn interpret(&self, rob : &mut T, desc : &mut D, code : &str) -> Vec<R>; 

    /// Interpret a file for a given robot
    fn interpret_file(&self, rob : &mut T, desc : &mut D, path : &str) -> Vec<R> {
        self.interpret(rob, desc, std::fs::read_to_string(path).unwrap().as_str())
    }
}