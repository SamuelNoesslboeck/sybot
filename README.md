# sybot

[![Crates.io version](https://img.shields.io/crates/v/sybot.svg?style=flat-square) ](https://crates.io/crates/sybot)
[![sybot: rustc 1.68+]][Rust 1.68]

[sybot: rustc 1.68+]: https://img.shields.io/badge/sybot-rustc_1.68+-lightgray.svg
[Rust 1.68]: https://blog.rust-lang.org/2023/03/09/Rust-1.68.0.html

A simple library to control groups of components and robots.

Extension library for the [syact](https://crates.io/crates/syact).

-- UNFINISHED DOCS -- 

Full documentation will be added soon

### Goal

- Create an all-in-one library for controlling robots, exposing them to networks and doing basic calculations

## In action

The following example creates a new [SyArm robot](https://github.com/SamuelNoesslboeck/SyArm_Mk1), runs all setup functions and executes a GCode-script. 

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
sybot = { version = "0.8.2, features = [ "rasp" ] }

# ...
```
</details>
<p></p>

```rust ,ignore
use sybot::{Robot, JsonConfig, ActRobot, Setup};
use sybot::robot::SyArm;
use sybot::intpr::Interpreter;
use sybot::intpr::gcode::init_intpr;

fn main() -> std::io::Result<()> {
    // Load the standard-partlibs in order to use motor names as data
    //
    // ```json
    // "ctrl": {
    //     "consts": "MOT_17HE15_1504S",    // Motor name, see
    // // <https://docs.rs/syact/0.11.1/syact/data/struct.StepperConst.html#associatedconstant.MOT_17HE15_1504S>
    //     "pin_dir": 17,
    //     "pin_step": 26
    // },
    // ```
    let libs = sybot::partlib::create_std_libs();

    // Create the robot out of the [configuration file]
    // (https://github.com/SamuelNoesslboeck/sybot/blob/master/res/SyArm_Mk1.conf.json)
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
```

*(Source: "examples/in_action.rs")*

## Features

For more features, see [syact#features](https://crates.io/crates/syact#features)

- [x] [Robots](/docs/robots.md)
  - [x] Basic 3D-Printer like robot ("Syomat")
  - [x] Basic robotic arm ("SyArm")
  - [x] [Custom robots](/docs/robots/custom_robots.md)
  - [x] [Tools](/docs/robots/tools.md)
- [ ] [Configuration files](/docs/configuration.md)
  - [ ] JSON
    - [x] Parsing
    - [ ] Generating
- [ ] [Calculation](/docs/calculation.md)
  - [ ] Forces
    - [x] Basic
    - [ ] Advanced
  - [ ] Inertias
    - [x] Basic
    - [ ] Advanced
  - [ ] Paths
- [x] [Controls](/docs/controls.md)
  - [x] GCode
  - [ ] Lua scripting
- [ ] [Networking](/docs/networking.md)
  - [x] HTTP
  - [x] MQTT
  - [ ] Serial
- [ ] Logging

## Issues and requests

If you encounter any issues or if you have any request for new features, feel free to create an issue at the [GitHub repo](https://github.com/SamuelNoesslboeck/sybot).