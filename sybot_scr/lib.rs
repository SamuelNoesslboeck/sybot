extern crate alloc;

// Submodules
    // #[cfg(feature = "gcode")]
    pub mod gcode;

    #[cfg(feature = "lua")]
    pub mod lua; 
// 

pub type Error = Box<dyn std::error::Error>;

/// Interpreters convert a string prompt into actions for the robot
pub trait Interpreter<ROB, DESC, RES> {
    /// Interpret a code string for a given robot
    fn interpret(&self, rob : &mut ROB, desc : &mut DESC, code : &str) -> Vec<RES>; 

    /// Interpret a file for a given robot
    fn interpret_file(&self, rob : &mut ROB, desc : &mut DESC, path : &str) -> Vec<RES> {
        self.interpret(rob, std::fs::read_to_string(path).unwrap().as_str())
    }
}