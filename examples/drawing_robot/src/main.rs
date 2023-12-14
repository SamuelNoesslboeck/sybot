use clap::{command, arg, value_parser};
use indicatif::ProgressBar;

use syact::prelude::*;
use sybot::prelude::*;

mod robot;
pub use robot::*;

pub const DRAW_SPEED_DEFAULT : SpeedFactor = unsafe {
    SpeedFactor::from_unchecked(0.25)
};

// Points
    pub const PIXEL_PER_MM : f32 = 4.0;

    #[derive(Copy, Clone, serde::Serialize, serde::Deserialize)]
    pub struct Line {
        p1 : [f32; 2],
        p2 : [f32; 2]
    }

    #[derive(serde::Serialize, serde::Deserialize)]
    pub struct LinesFile {
        contour : Vec<Line>
    }

    pub fn load_points(path : &str) -> LinesFile {
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
    }

    pub fn convert_pixel(pixel : f32) -> Phi {
        Phi(pixel / PIXEL_PER_MM)
    }

    pub fn convert_line(line : Line) -> [[Phi; 2]; 2] {
        [
            [ convert_pixel(line.p1[0]), convert_pixel(line.p1[1]) ],
            [ convert_pixel(line.p2[0]), convert_pixel(line.p2[1]) ]
        ]
    }
// 

fn main() {
    // Init logging
        env_logger::init();
    // 

    // Cmd
        let matches = command!() 
            .about("Drawing robot system")
            .arg(arg!([path] "Pin number of the step pin").value_parser(value_parser!(String)))
            .arg(arg!([z_state] "The current state of the Z-Axis (Drawing 0, Lifted 1)").value_parser(value_parser!(usize)))
            .get_matches();

        let path : String = matches.get_one::<String>("path").expect("A valid path has to be provided").clone();
        let z_state : usize = *matches.get_one("z_state").expect("A valid Z-State has to be provided");

        let draw_speed = std::env::var("DRAW_SPEED").map(|s| s.parse::<SpeedFactor>().unwrap()).unwrap_or(DRAW_SPEED_DEFAULT);
    // 

    // RDS
        let mut rob = linear_xy_robot_new();

        // let desc = LinearXYDescriptor::new();

        let mut stat = LinearXYStation::new();
    // 

    // Lines
        let lines = load_points(path.as_str());
    // 

    // Init
    rob.comps_mut().set_config(StepperConfig::new(12.0, None));
    rob.comps_mut().apply_inertias(&WEIGHT_AXES);
    rob.setup().unwrap();

    stat.z_axis.set_state(z_state);
    stat.z_axis.apply_inertia(WEIGHT_BED);
    stat.z_axis.setup().unwrap();

    println!("Driving to home position ... ");

    stat.home(&mut rob).unwrap();

    println!("Starting to draw ... ");

    let pb = ProgressBar::new(lines.contour.len() as u64);

    // Safe to use
    let mut last_point = unsafe { core::mem::zeroed() };

    if let Some(&init_line) = lines.contour.first() {
        let [ p1, _ ] = convert_line(init_line);
        stat.reposition_pen(&mut rob, p1).unwrap();   
        last_point = p1;
    }

    for line in lines.contour {
        let [ p1, p2 ] = convert_line(line);

        if p1 != last_point {
            stat.reposition_pen(&mut rob, p1).unwrap();
        }

        log::debug!("Driving to {:?}", p2);
        rob.move_abs_j(p2, draw_speed).unwrap();
        rob.await_inactive().unwrap();
        
        last_point = p2;

        pb.inc(1);
    }

    pb.finish_with_message("done");

    stat.home(&mut rob).unwrap();
}