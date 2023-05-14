use glam::Vec3;
use stepper_lib::math::PathBuilder;
use stepper_lib::units::*;

pub fn split_linear(pos_0 : Vec3, delta_pos : Vec3, split_len : f32) -> Vec<Vec3> {
    let n_split = (delta_pos.length() / split_len).ceil() as usize;
    let delta = delta_pos / n_split as f32;

    let mut pos_vec = vec![ ];

    for i in 0 ..=n_split {
        pos_vec.push(pos_0 + delta * i as f32);
    }

    pos_vec
}

// pub fn convert_to_phis<const C : usize>(robot : &impl ActRobot<C>, deco : &[f32], pos_vec : &Vec<Vec3>) -> Result<Vec<[Phi; C]>, crate::Error> {
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