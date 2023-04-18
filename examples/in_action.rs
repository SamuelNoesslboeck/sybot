use sybot_lib::ActRobot;
use sybot_lib::conf::JsonConfig;
use sybot_lib::intpr::Interpreter;
use sybot_lib::intpr::gcode::init_intpr;
use sybot_lib::robot::SyArm;

fn main() -> std::io::Result<()> {
    // Load the standard-partlibs in order to use motor names as data
    //
    // ```json
    // "ctrl": {
    //     "consts": "MOT_17HE15_1504S",    // Motor name, see
    // // <https://docs.rs/stepper_lib/0.11.1/stepper_lib/data/struct.StepperConst.html#associatedconstant.MOT_17HE15_1504S>
    //     "pin_dir": 17,
    //     "pin_step": 26
    // },
    // ```
    let libs = sybot_lib::partlib::create_std_libs();

    // Create the robot out of the [configuration file]
    // (https://github.com/SamuelNoesslboeck/sybot_lib/blob/master/res/SyArm_Mk1.conf.json)
    let mut syarm = SyArm::from_conf(
        JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
    )?;

    // Run setup functions
    syarm.setup();
    // Enables async movements (multiple motors moving at once)
    syarm.setup_async();

    // DEBUG
        // Select "NoTool" at index 2
        syarm.set_tool_id(2);
    // 

    // Create a new GCode interpreter
    let intpr = init_intpr();

    // Run a GCode script
    dbg!(intpr.interpret_file(&mut syarm, "res/gcode/basicYZpos.gcode"));

    Ok(())
}