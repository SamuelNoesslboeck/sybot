use indicatif::ProgressBar;

use syact::prelude::*;
use sybot::prelude::*;

mod robot;
pub use robot::*;

// Points
    pub const PIXEL_PER_MM : f32 = 4.0;

    #[derive(serde::Serialize, serde::Deserialize)]
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

    // RDS
        let mut rob = linear_xy_robot_new();

        // let desc = LinearXYDescriptor::new();

        let mut station = LinearXYStation { };
    // 

    // Lines
        let lines = load_points("assets/sample_lines.json");
    // 

    rob.comps_mut().set_config(StepperConfig::new(12.0, None));
    rob.setup().unwrap();

    println!("Driving to home position ... ");

    station.home(&mut rob).unwrap();
    
    let mut buffer;
    loop {
        println!("X: ");

        buffer = String::new();
        std::io::stdin().read_line(&mut buffer).unwrap();

        let x = buffer.trim().parse::<Phi>().unwrap();

        println!("Y: ");

        buffer = String::new();
        std::io::stdin().read_line(&mut buffer).unwrap();

        let y = buffer.trim().parse::<Phi>().unwrap();

        rob.move_abs_j([ x, y ], SpeedFactor::from(0.25)).unwrap();
        rob.await_inactive().unwrap();
    }
}