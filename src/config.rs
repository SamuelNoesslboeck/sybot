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
        /// An empty `AngleConfig`, with all values set to `0`/´false`
        pub const EMPTY : Self = Self { offset: Delta::ZERO, counter: false };

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

// AxisConf
    /// Defines the way a robot should act when there is more than one possible way of accessing an object
    pub trait AxisConfig {
        /// Returns the phis stored
        fn phis<'a>(&'a self) -> &'a [Phi];

        /// Configure the configuration by altering the `Phi` values
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