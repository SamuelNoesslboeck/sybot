use core::fmt::Debug;

use syact::units::*;

// Submodules
    mod device;
    pub use device::*;

    mod seg;
    pub use seg::*;
// 

// RDS
    mod desc;
    pub use desc::*;

    mod rob;
    pub use rob::*;

    mod stat;
    pub use stat::*;
// 

#[derive(Clone, Debug)]
pub struct Mode {
    pub name : String,
    pub desc : String,

    pub speed_f : f32
}

impl Mode {
    pub fn new<N : Into<String>, D : Into<String>>(name : N, desc : D, speed_f : f32) -> Self {
        Self { name : name.into(), desc: desc.into(), speed_f }
    }
}

pub fn default_modes() -> [Mode; 2] { 
    [
        Mode {
            name: String::from("Setup"), 
            desc: String::from("Mode with decreased speeds used for setting up"),
            speed_f : 0.5
        },
        Mode {
            name: String::from("Auto"), 
            desc: String::from("Mode used for running automated programms with full speed"),
            speed_f : 1.0
        }
    ]
}

#[derive(Clone, Debug)]
pub struct Vars<const C : usize> {
    pub phis : [Phi; C],
}

impl<const C : usize> Vars<C> {
    pub fn cache_phis(&self, phis_opt : [Option<Phi>; C]) -> [Phi; C] {
        let mut phis = self.phis;

        for i in 0 .. C {
            if let Some(phi) = phis_opt[i] {
                phis[i] = phi;
            }
        }

        phis
    }
}

impl<const C : usize> Default for Vars<C> {
    fn default() -> Self {
        Self {
            phis: [Phi::default(); C]
        }
    }
}