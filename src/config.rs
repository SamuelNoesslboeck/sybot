use syunit::*;

// Angle Configuration
    /// Angle configuration (phi to gamma conversion), allowing 
    /// - `offset`: The `Phi` value has an offset compared to the `Gamma` value
    /// - `counter`: The `Phi` value is working as a counter angle to `Gamma` (The `Gamma` value will be negated)
    #[derive(Debug, Default, Clone, Copy)]
    pub struct AngleConfig {
        /// Offset of the value
        pub offset : Delta,
        /// Wheiter or not the angle is a counterpart (negative addition)
        pub counter : bool
    }

    impl AngleConfig {
        /// Convert the given gamma angle to a phi angle
        pub fn phi_from_gamma(&self, gamma : Gamma) -> Phi {
            (if self.counter { 
                -gamma
            } else { 
                gamma
            } + self.offset).force_to_phi()
        }
        
        /// Convert the given phi angle to a gamma angle
        pub fn gamma_from_phi(&self, phi : Phi) -> Gamma {
            if self.counter { 
                -phi.force_to_gamma() + self.offset
            } else { 
                phi.force_to_gamma() - self.offset
            }
        }
    }
//

// Mode
    #[derive(Clone, Debug)]
    pub struct Mode {
        pub name : String,
        pub desc : String,

        pub speed_f : f32
    }

    impl Mode {
        pub fn new<N : Into<String>, D : Into<String>>(name : N, desc : D, speed_f : f32) -> Self {
            Self { 
                name: name.into(), 
                desc: desc.into(), 
                speed_f 
            }
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
// 

// AxisConf
    pub trait AxisConfig {
        fn phis<'a>(&'a self) -> &'a [Phi];

        fn configure(&mut self, phis : Vec<Phi>) -> Result<(), crate::Error>; 
    }

    impl AxisConfig for () {
        fn phis<'a>(&'a self) -> &'a [Phi] {
            &[]
        }

        fn configure(&mut self, _ : Vec<Phi>) -> Result<(), crate::Error> {
            Ok(())
        }
    }
// 