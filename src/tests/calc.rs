use super::*;

mod postion
{
    use super::*;

    #[test]
    fn double_convert() {
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        );

        let pos = Vec3::new(0.0, 350.0, 400.0);
        
        let angles = syarm.get_with_fixed_dec(pos, 0.0);
        let points = syarm.points_by_phis(&angles);

        assert!((pos - points[3]).length() > f32::EPSILON);
    }

    #[test]
    fn single_convert() {
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        );
        
        let angles = [ 0.0, PI / 2.0, -PI / 2.0, 0.0 ];
        let points = syarm.points_by_phis(&angles);

        dbg!(points);
    }

    #[test]
    fn angles_for_components_without_meas() {
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        );

        let angles = [ 0.0, PI / 2.0, -PI / 2.0, 0.0 ];
        let gammas = syarm.gammas_for_phis(angles);
        
        assert!(syarm.valid_gammas(gammas), "The gammas generated are not valid! Gammas: {:?}, Valids: {:?}", gammas, syarm.valid_gammas_verb(gammas));
    }    

    #[test]
    fn angles_for_components_with_meas() {
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        );

        let angles = [ 0.0, PI / 2.0, -PI / 2.0, 0.0 ];
        let gammas = syarm.gammas_for_phis(angles);

        syarm.measure(10).unwrap(); 
        
        assert!(syarm.valid_gammas(gammas), "The gammas generated are not valid! Gammas: {:?}, Valids: {:?}", gammas, syarm.valid_gammas_verb(gammas));
    }
}

mod load
{
    use super::*;
    
    #[test]
    fn test_arm() {
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        );

        const ANGLES : Phis = [0.0, PI / 2.0, -PI / 2.0, 0.0];

        syarm.write_position(&syarm.gammas_for_phis(ANGLES)); 

        syarm.update_sim();

        dbg!(syarm.get_inertias(&syarm.vectors_by_phis(&ANGLES)));
    }
}