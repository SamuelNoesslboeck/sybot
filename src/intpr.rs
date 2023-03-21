// Submodules
pub mod gcode;

pub mod gfuncs;

pub trait Interpreter<ROB, RES> {
    fn interpret(&self, rob : &mut ROB, code : &str) -> Vec<RES>; 

    fn interpret_file(&self, rob : &mut ROB, path : &str) -> Vec<RES> {
        self.interpret(rob, std::fs::read_to_string(path).unwrap().as_str())
    }
}