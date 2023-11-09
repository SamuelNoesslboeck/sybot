use syact::units::*;

// AxisConf
    /// Angle config (phi to gamma conversion)
    #[derive(Debug, Default, Clone, Copy)]
    pub struct AngConf {
        /// Offset of the value
        pub offset : Delta,
        /// Wheiter or not the angle is a counterpart (negative addition)
        pub counter : bool
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
    pub trait AxisConf {
        fn phis<'a>(&'a self) -> &'a [Phi];

        fn configure(&mut self, phis : Vec<Phi>) -> Result<(), crate::Error>; 
    }

    #[derive(Clone, Debug, Default)]
    pub struct EmptyConf { }

    impl AxisConf for EmptyConf {
        fn phis<'a>(&'a self) -> &'a [Phi] {
            &[]
        }

        fn configure(&mut self, _ : Vec<Phi>) -> Result<(), crate::Error> {
            Ok(())
        }
    }
// 