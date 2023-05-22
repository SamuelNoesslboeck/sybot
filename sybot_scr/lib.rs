extern crate alloc;

// Submodules
    #[cfg(feature = "lua")]
    pub mod lua; 
// 

/// Interpreters convert a string prompt into actions for the robot
pub trait Interpreter<ROB, RES> {
    /// Interpret a code string for a given robot
    fn interpret(&self, rob : &mut ROB, code : &str) -> Vec<RES>; 

    /// Interpret a file for a given robot
    fn interpret_file(&self, rob : &mut ROB, path : &str) -> Vec<RES> {
        self.interpret(rob, std::fs::read_to_string(path).unwrap().as_str())
    }
}