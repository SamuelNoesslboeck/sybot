use super::*;

use glam::Vec3;

mod postion
{
    use stepper_lib::units::*;

    use super::*;

    #[test]
    fn double_convert() -> std::io::Result<()> {
        let libs = crate::partlib::create_std_libs();
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
        )?;

        dbg!(syarm.set_tool_id(
            syarm.json_conf().unwrap().tool_id_by_name("No_Tool").unwrap()
        ));

        let positions = [
            Vec3::new(0.0, 330.0, 400.0),
            Vec3::new(0.0, -330.0, 400.0),
            Vec3::new(330.0, 0.0, 400.0),
            Vec3::new(-330.0, 0.0, 400.0),

            Vec3::new(200.0, 200.0, 400.0),
            Vec3::new(-100.0, 300.0, 400.0),
        ];
        
        for pos in positions {
            let angles = syarm.phis_from_vec(pos, [ 0.0 ]);
            let points = syarm.points_from_phis(&angles);

            println!("{:?}", angles);
            println!("Original: {:?} | Generated: {:?}", pos, points[3]);
    
            assert!((pos - points[3]).length() < 0.01); // 0.01 allow a small tolerance
        }

        Ok(())
    }

    #[test]
    fn single_convert() -> std::io::Result<()>  {
        let libs = crate::partlib::create_std_libs();
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
        )?;
        
        let phis = [ Phi::ZERO, Phi(PI / 2.0), Phi(-PI / 2.0), Phi::ZERO ];
        let points = syarm.points_from_phis(&phis);

        dbg!(points);

        Ok(())
    }

    #[test]
    fn angles_for_components_without_meas() -> std::io::Result<()>  {
        let libs = crate::partlib::create_std_libs();
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
        )?;

        let phis = [ Phi::ZERO, Phi(PI / 2.0), Phi(-PI / 2.0), Phi::ZERO ];
        let gammas = syarm.gammas_from_phis(phis);
        
        assert!(syarm.check_gammas(gammas).is_ok(), "The gammas generated are not valid! Gammas: {:?}, Valids: {:?}", gammas, syarm.check_gammas(gammas));

        Ok(())
    }    

    #[test]
    fn angles_for_components_with_meas() -> std::io::Result<()> {
        let libs = crate::partlib::create_std_libs();
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
        )?;

        let phis = [ Phi::ZERO, Phi(PI / 2.0), Phi(-PI / 2.0), Phi::ZERO ];
        let gammas = syarm.gammas_from_phis(phis);

        syarm.measure()?; 
        
        assert!(syarm.check_gammas(gammas).is_ok(), "The gammas generated are not valid! Gammas: {:?}, Valids: {:?}", gammas, syarm.check_gammas(gammas));

        Ok(())
    }
}

mod load
{
    use stepper_lib::units::*;

    use super::*;
    
    #[test]
    fn inertias() -> std::io::Result<()> {
        let libs = crate::partlib::create_std_libs();
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
        )?;

        const PHIS : [Phi; 4] = [ Phi::ZERO, Phi(PI / 2.0), Phi(-PI / 2.0), Phi::ZERO ];

        syarm.write_gammas(&syarm.gammas_from_phis(PHIS)); 

        syarm.update(None)?;

        dbg!(syarm.inertias_from_phis(&PHIS));

        Ok(())
    }
}