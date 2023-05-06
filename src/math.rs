use glam::Vec3;
use stepper_lib::units::*;

use crate::ActRobot;

pub fn split_linear(pos_0 : Vec3, delta_pos : Vec3, split_len : f32) -> Vec<Vec3> {
    let n_split = (delta_pos.length() / split_len).ceil() as usize;
    let delta = delta_pos / n_split as f32;

    let mut pos_vec = vec![ ];

    for i in 0 ..=n_split {
        pos_vec.push(pos_0 + delta * i as f32);
    }

    pos_vec
}

pub fn convert_to_phis<const C : usize>(robot : &impl ActRobot<C>, deco : &[f32], pos_vec : &Vec<Vec3>) -> Result<Vec<Phi>, crate::Error> {
    let phis_vec = vec![]; 

    for pos in pos_vec {
        let reduced = robot.reduce_to_def(*pos, deco);
        phis_vec.push(robot.phis_from_def_vec(reduced));
    }

    phis_vec
}