use super::*;

use glam::Vec3;

mod postion
{
    use super::*;

    #[test]
    fn double_convert() -> std::io::Result<()> {
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        )?;

        let pos = Vec3::new(0.0, 350.0, 400.0);
        
        let angles = syarm.phis_from_vec(pos, 0.0);
        let points = syarm.points_from_phis(&angles);

        assert!((pos - points[3]).length() > f32::EPSILON);

        Ok(())
    }

    #[test]
    fn single_convert() -> std::io::Result<()>  {
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        )?;
        
        let angles = [ 0.0, PI / 2.0, -PI / 2.0, 0.0 ];
        let points = syarm.points_from_phis(&angles);

        dbg!(points);

        Ok(())
    }

    #[test]
    fn angles_for_components_without_meas() -> std::io::Result<()>  {
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        )?;

        let angles = [ 0.0, PI / 2.0, -PI / 2.0, 0.0 ];
        let gammas = syarm.gammas_from_phis(angles);
        
        assert!(syarm.valid_gammas(&gammas).is_ok(), "The gammas generated are not valid! Gammas: {:?}, Valids: {:?}", gammas, syarm.valid_gammas(&gammas));

        Ok(())
    }    

    #[test]
    fn angles_for_components_with_meas() -> std::io::Result<()> {
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        )?;

        let angles = [ 0.0, PI / 2.0, -PI / 2.0, 0.0 ];
        let gammas = syarm.gammas_from_phis(angles);

        syarm.measure(10).unwrap(); 
        
        assert!(syarm.valid_gammas(&gammas).is_ok(), "The gammas generated are not valid! Gammas: {:?}, Valids: {:?}", gammas, syarm.valid_gammas(&gammas));

        Ok(())
    }
}

mod load
{
    use super::*;
    
    #[test]
    fn inertias() -> std::io::Result<()> {
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        )?;

        const ANGLES : Phis<4> = [0.0, PI / 2.0, -PI / 2.0, 0.0];

        syarm.write_gammas(&syarm.gammas_from_phis(ANGLES)); 

        syarm.update(None);

        dbg!(syarm.inertias_from_phis(&ANGLES));

        Ok(())
    }
}