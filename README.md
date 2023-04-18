# sybot_lib

[![Crates.io version](https://img.shields.io/crates/v/sybot_lib.svg?style=flat-square) ](https://crates.io/crates/sybot_lib)
[![sybot_lib: rustc 1.68+]][Rust 1.68]

[sybot_lib: rustc 1.68+]: https://img.shields.io/badge/sybot_lib-rustc_1.68+-lightgray.svg
[Rust 1.68]: https://blog.rust-lang.org/2023/03/09/Rust-1.68.0.html

A simple library to control groups of components and robots.

Extension library for the [stepper_lib](https://crates.io/crates/stepper_lib).

-- UNFINISHED DOCS -- 

Fully documentation will be added soon

### Goal

- Create an all-in-one library for controlling robots, expose them to networks and do basic calculations

# In action

The following example creates a new [SyArm robot](https://github.com/SamuelNoesslboeck/SyArm_Mk1), runs all setup functions and executes a GCode-script. 

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
sybot_lib = { version = "0.7.8", features = [ "rasp" ] }

# ...
```
</details>
<p></p>

```rust
use sybot_lib::{SyArm, Robot, JsonConfig};
use sybot_lib::intpr::Interpreter;
use sybot_lib::intpr::gcode::init_intpr;

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
```

*(Source: "examples/in_action.rs")*

# Overview

- [Features](#features)
- [Robots](#robots)
  - [Custom robots](#custom-robots)
  - [Tools](#tools)
- [Configuration](#configuration)
- [Calculation](#calculation)
- [Command systems](#command-systems)
- [Networking](#networking)
  - [Remotes](#remotes)
  - [HTTP](#http)
  - [MQTT](#mqtt)
  - [Custom connections](#custom-connections)
- [Platforms](#issues)

## Features

For more features, see [stepper_lib#features](https://crates.io/crates/stepper_lib#features)

- [x] Robots
  - [x] Basic 3D-Printer like robot ("Syomat")
  - [x] Basic robotic arm ("SyArm")
  - [x] Custom robots
  - [x] Tools
  - [ ] JSON configuration files
    - [x] Parsing
    - [ ] Generating
- [ ] Calculation
  - [ ] Forces
    - [x] Basic
    - [ ] Advanced
  - [ ] Inertias
    - [x] Basic
    - [ ] Advanced
  - [ ] Paths
- [x] Controls
  - [x] GCode
- [ ] Connections
  - [x] HTTP
  - [x] MQTT
  - [ ] Serial
- [ ] Logging

## Robots

### Custom robots

### Tools

## Configuration

The library includes methods for parsing JSON configuration files (file extension ".conf.json"). Out of these files, all the constants for a previously defined robot can be parsed. 

Example configuration file:

```json
{
  "name": "SyArm_Mk1",
  "conf_version": "0.0.1/2023/02/21",
  "author": "Samuel Nösslböck",

  "lk": {
    "u": 12,
    "s_f": 1.5
  },

  "anchor": [ 0.0, 0.0, 100.0 ],
  "dims": [
    [ 0.0, 0.0, 15.0 ],
    [ 0.0, 285.0, 0.0 ],
    [ 0.0, 285.0, 0.0 ],
    [ 0.0, 45.0, 0.0 ]
  ],
  "axes": [
    [ 0.0, 0.0, 1.0 ],
    [ 1.0, 0.0, 0.0 ],
    [ 1.0, 0.0, 0.0 ],
    [ 1.0, 0.0, 0.0 ]
  ],

  "comps": [
    {
      "name": "Base",
      "type_name": "stepper_lib::comp::gear_bearing::GearJoint",
      "obj": {
        "ctrl": {
          "consts": "MOT_17HE15_1504S",
          "pin_dir": 17,
          "pin_step": 26
        },
        "ratio": 0.08333
      },
      "sim": {
        "mass": 0.2,
        "fric": 2.0
      },
      "meas": {
        "pin": 16,
        "set_val": 0.0,
        "dist": 0.0
      },
      "limit": {
        "vel": 5.0,
        "min": -3.14,
        "max": 3.14
      }
    },
// ... 
```

## Calculation

The library includes functions for inertia and load calculations.

## Command systems

With the `intpr` module the user can create own command systems or use the already implemented GCode one.

## Networking

### Remotes

### HTTP

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi and with http enabled
sybot_lib = { version = "0.7.8", features = [ "rasp", "http" ] }

# ...
```
</details>
<p></p>

```rust
extern crate alloc;

use core::cell::RefCell;

use alloc::rc::Rc;
use actix_web::{HttpServer, App};

use sybot_lib::{JsonConfig, SyArm, Robot};
use sybot_lib::intpr::gcode::init_intpr;
use sybot_lib::http::create_robot_webserver;

#[actix::main]
async fn main() -> Result<(), std::io::Error> {    
    HttpServer::new(move || {
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
        
        // Create a new interpreter in a [std::rc::Rc]
        let intpr = Rc::new(RefCell::new(init_intpr()));

        // Create the webserver
        create_robot_webserver::<SyArm, _, 4, 1, 4, 4>(syarm, intpr, App::new())
    }).bind(("127.0.0.1", 8080))?   // Bind the webserver 
    .run()
    .await
}
```

### MQTT

### Custom connections

With the creation of custom remotes, the user can create their own connections to the internet or other forms of networks.

## Platforms and simulation

The final goal of the library is to work on as many platforms as possible. To configure the library for a specific platform, the right features have to be enabled. 

The current platforms and features enabled are
- "rasp": Raspberry Pi and similar controllers

```toml
# platform features
rasp = [ "stepper_lib/rasp" ]
```

If no platform is selected the library automatically goes into simulation mode. In simulation mode, no movements will be executed, but all calculations will be done. Which means that for example GCode scripts can be "debugged" in advance.

## Issues and requests

If you encounter any issues or if you have any request for new features, feel free to create an issue at the [GitHub repo](https://github.com/SamuelNoesslboeck/sybot_lib).