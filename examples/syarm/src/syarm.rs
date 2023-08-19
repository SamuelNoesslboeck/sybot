use core::f32::consts::PI;

use glam::{Vec3, Mat3};
use serde::{Deserialize, Serialize};

use syact::prelude::*;
use sybot::prelude::*;

// Robot
    #[derive(StepperCompGroup, Deserialize, Serialize)]
    pub struct SyArmComps {
        pub base : Gear<Stepper>,
        pub arm1 : CylinderTriangle<Stepper>,
        pub arm2 : CylinderTriangle<Stepper>,
        pub arm3 : Gear<Stepper>
    }

    pub type SyArmRob = StepperRobot<SyArmComps, 4>;
// 

// Descriptor
    #[derive(Default, Debug)]
    pub struct SyArmConf {
        pub phis : [Phi; 1]
    }

    impl AxisConf for SyArmConf {
        fn phis<'a>(&'a self) -> &'a [Phi] {
            &self.phis
        }

        fn configure(&mut self, phis : Vec<Phi>) -> Result<(), sybot::Error> {
            if phis.len() < 1 {
                Err("Not enough angles for configuring the axis configuration! (1 required)".into())
            } else {
                self.phis[0] = phis[0];
                Ok(())
            }
        }
    }

    #[derive(Debug)]
    pub struct SyArmDesc {    
        pub segments : LinSegmentChain<4>,

        _tcp : PointRef,
        _wobj : WorldObj,
        _aconf : SyArmConf,
    }

    impl SyArmDesc {
        fn new(mut wobj : WorldObj, segments : &Vec<SegmentInfo>) -> Result<Self, sybot::Error> {
            let tcp = PointRef::new(Position::new(Vec3::ZERO));
            wobj.add_point("tcp", tcp.clone());

            Ok(Self {
                segments: LinSegmentChain::from_wobj(segments, &mut wobj, "tcp")?,

                _tcp: tcp,
                _wobj: wobj,
                _aconf: SyArmConf::default()
            })
        }

        pub fn base<'a>(&'a self) -> &'a Segment {
            &self.segments[0]
        }

        pub fn arm1<'a>(&'a self) -> &'a Segment {
            &self.segments[1]
        }

        pub fn arm2<'a>(&'a self) -> &'a Segment {
            &self.segments[2]
        }

        pub fn arm3<'a>(&'a self) -> &'a Segment {
            &self.segments[3]
        }
    }

    impl TryFrom<DescPackage> for SyArmDesc {
        type Error = sybot::Error;

        fn try_from(value: DescPackage) -> Result<Self, Self::Error> {
            Self::new(
                value.rcs.ok_or("A valid RCS must be provided for the robot!")?, 
                value.segments.as_ref().ok_or("A vaild set of segments must be provided for the robot!")?
            )
        }
    }

    impl Descriptor<4> for SyArmDesc {
        // Axis config
            fn aconf<'a>(&'a self) -> &'a dyn AxisConf {
                &self._aconf
            }

            fn aconf_mut<'a>(&'a mut self) -> &'a mut dyn AxisConf {
                &mut self._aconf
            }
        //

        // World object
            #[inline]
            fn wobj<'a>(&'a self) -> &'a WorldObj {
                &self._wobj
            }

            #[inline]
            fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj {
                &mut self._wobj
            }

            #[inline]
            fn tcp(&self) -> &PointRef {
                &self._tcp
            }

            fn cache_tcp(&self, x_opt : Option<f32>, y_opt : Option<f32>, z_opt : Option<f32>) -> Vec3 {
                let pos = self._tcp.pos();

                Vec3::new(
                    x_opt.unwrap_or(pos.x), 
                    y_opt.unwrap_or(pos.y), 
                    z_opt.unwrap_or(pos.z)
                )
            }
        // 

        // Events
            fn update<R : Robot<4, Comp = S, CompGroup = G>, S : SyncComp + ?Sized + 'static, G : SyncCompGroup<S, 4>>(&mut self, _ : &mut R, phis : &[Phi; 4]) -> Result<(), sybot::Error> {
                self.segments.update(phis)?;
                
                let tcp_new = self.segments.calculate_end();
                let mut tcp = self._tcp.borrow_mut();

                *(tcp.pos_mut()) = *tcp_new.pos();
                *(tcp.ori_mut()) = *tcp_new.ori();

                Ok(())
            }
        // 

        // Calculate
            fn convert_pos<R : Robot<4, Comp = S, CompGroup = G>, S : SyncComp + ?Sized, G>(&mut self, rob : &mut R, mut pos : Position) -> Result<[Phi; 4], sybot::Error> {
                let phi_b = sybot::math::full_atan(pos.x(), pos.y());
                let dec_ang = self.aconf().phis()[0].0;

                let z_matr = Mat3::from_rotation_z(phi_b);
                let mut tcp_vec = self.segments.tcp().pos();
                tcp_vec = z_matr * tcp_vec;
                tcp_vec = Mat3::from_rotation_y(-dec_ang) * tcp_vec;

                pos.shift(-tcp_vec);
                pos.shift(-*self._wobj.pos());
                pos.shift(-self.segments[0].pos()); 
                pos.transform(Mat3::from_rotation_z(-phi_b)); 
                pos.shift(-self.segments[1].pos());
    
                let arm2 = self.arm2().pos();
                let arm3 = self.arm3().pos();

                let (alpha_2, _, gamma_2) = 
                    sybot::math::calc_triangle(arm2.length(), arm3.length(), pos.pos().length()); 

                let mut pos_ang = Vec3::X.angle_between(*pos.pos());

                if pos.z() < 0.0 {
                    pos_ang = -pos_ang;
                }

                let phi_1 = alpha_2 + pos_ang;
                let phi_2 = gamma_2 - PI;
                let phis = [ Phi(phi_b), Phi(phi_1), Phi(phi_2), Phi(dec_ang - phi_1 - phi_2) ];

                rob.valid_phis(&phis)?;

                Ok(phis) 
            }
        // 
    }
// 

// Station
    pub struct SyArmStation { 
        pub mdata : [SimpleMeasData; 4]
    }

    impl TryFrom<StationPackage> for SyArmStation {
        type Error = sybot::Error;

        fn try_from(pkg : StationPackage) -> Result<Self, Self::Error> {
            let mdata : [SimpleMeasData; 4] = pkg.mdata
                .ok_or("Expected measurment data for station!")?
                .try_into()
                .map_err(|err : Vec<SimpleMeasData>| -> crate::Error {
                    format!("Wrong number of meas-data entries! (4 expected, {} received)", err.len()).into()
                })?;

            Ok(Self { 
                mdata
            })
        }
    }

    impl Station for SyArmStation {
        fn init_meas(&mut self) {
            
        }
    }
// 