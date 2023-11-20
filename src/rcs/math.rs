use core::f32::consts::PI;

use glam::Vec3;
use syact::units::*;

use crate::rcs::Point;

pub fn sub_phis<const C : usize>(a : [Phi; C], b : [Phi; C]) -> [Delta; C] {
    let mut deltas = [Delta::ZERO; C];
    for i in 0..C {
        deltas[i] = a[i] - b[i];
    }
    deltas
}

pub fn law_of_cosines(a : f32, b : f32, c : f32) -> f32 {
    ((a.powi(2) + b.powi(2) - c.powi(2)) / 2.0 / a / b).acos()
}

#[inline]
pub fn law_of_sines(a : f32, c : f32, gamma : f32) -> f32 {
    (a * gamma.sin() / c).asin()
}

#[inline]
pub fn comple_triangle(alpha : f32, gamma : f32) -> f32 {
    PI - alpha - gamma
}

pub fn calc_triangle(a : f32, b : f32, c : f32) -> (f32, f32, f32) {
    let gamma = law_of_cosines(a, b, c);
    let alpha = law_of_sines(a, c, gamma);
    let beta = comple_triangle(alpha, gamma);

    ( alpha, beta, gamma )
}

pub fn calc_triangle_vec(c_p : Vec3, b_p : Vec3) -> (f32, f32, f32) {
    let b = c_p.length();
    let a = b_p.length();
    let c = (b_p + c_p).length();

    calc_triangle(a, b, c)
}

pub fn calc_triangle_pos(c_p : &dyn Point, b_p : &dyn Point) -> (f32, f32, f32) {
    let b = c_p.pos().length();
    let a = b_p.pos().length();
    let c = c_p.to_higher_system(*b_p.pos()).length();

    calc_triangle(a, b, c)
}

pub fn split_linear(pos_0 : Vec3, delta_pos : Vec3, split_len : f32) -> Vec<Vec3> {
    let n_split = (delta_pos.length() / split_len).ceil() as usize;
    let delta = delta_pos / n_split as f32;

    let mut pos_vec = vec![ ];

    for i in 0 ..=n_split {
        pos_vec.push(pos_0 + delta * i as f32);
    }

    pos_vec
}

// pub fn convert_to_phis<const C : usize>(robot : &impl ActRobot<C>, deco : &[f32], pos_vec : &Vec<Vec3>) 
// -> Result<Vec<[Phi; C]>, crate::Error> {
//     let mut phis_vec = vec![]; 

//     for pos in pos_vec {
//         let reduced = robot.reduce_to_def(*pos, deco)?;
//         phis_vec.push(robot.phis_from_def_vec(reduced));
//     }

//     Ok(phis_vec)
// }

// pub fn build_path<const C : usize>(robot : &impl ActRobot<C>, phis : &Vec<[Phi; C]>) {
//     let mut builder : PathBuilder<C> = robot.comps().get_pathbuilder([Omega::ZERO; C]);
    
//     let mut tstack = vec![];
//     let mut dstack = vec![];

//     for i in 0 .. (phis.len() - 1) { 
//         let mut deltas = [Delta::ZERO; C];

//         for n in 0 .. C {
//             deltas[n] = phis[i + 1][n] - phis[i][n];
//         }

//         builder.next_all(&mut tstack, &mut dstack, i);
//     }
// }