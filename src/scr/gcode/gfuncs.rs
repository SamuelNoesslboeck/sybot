use std::process::exit;

use syact::units::*;
use crate::rcs::Position;
use super::*;

// General functions
    /// G0 X{Position} Y{Position} Z{Position} DECO{Angle} \
    /// Rapid positioning
    pub fn g0<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (robot : &mut R, desc : &mut D, _ : &mut S, code : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let pos = desc.cache_tcp(
            arg_by_letter(args, 'X'), 
            arg_by_letter(args, 'Y'), 
            arg_by_letter(args, 'Z')
        );

        let f_speed = arg_by_letter(args, 'S').unwrap_or(1.0);

        let deltas = if code.minor_number() == 0 {
            robot.move_p_sync(desc, Position::new(pos), f_speed)?
        } else if code.minor_number() == 1 {
            // let c_rob = robot.complex_rob_mut();

            // robot.move_j_abs_async(robot.gammas_from_phis(phis))?;
            // robot.await_inactive()?
            todo!()
        } else {
            // Create error!
            panic!("Bad minor number!");
        };

        robot.update()?;

        let phis = robot.phis();
        desc.update(robot, &phis)?;
        
        Ok(serde_json::json!({ 
            "phis": Vec::from(phis),
            "deltas": Vec::from(deltas)
        }))
    }

    pub fn g1<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (robot : &mut R, desc : &mut D, _ : &mut S, _ : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let _pos = desc.cache_tcp(
            arg_by_letter(args, 'X'), 
            arg_by_letter(args, 'Y'), 
            arg_by_letter(args, 'Z')
        );

        let _f_speed = arg_by_letter(args, 'S').unwrap_or(1.0);

        

        robot.update()?;

        let phis = robot.phis();
        desc.update(robot, &phis)?;
        
        Ok(serde_json::json!({ 
            "phis": Vec::from(phis),
            // "deltas": Vec::from(deltas)
        }))
    }
    /// G4 X{Seconds} P{Milliseconds}
    /// Dwell (sleeping)
    pub fn g4<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (_ : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let seconds = 
            arg_by_letter(args, 'X').unwrap_or(0.0)            // Seconds
            + arg_by_letter(args, 'P').unwrap_or(0.0)/1000.0;  // Milliseconds
        std::thread::sleep(core::time::Duration::from_secs_f32(seconds));
        Ok(serde_json::json!(seconds))
    }

    /// G28 \
    /// Return to home position
    pub fn g28<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (robot : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        match robot.auto_meas() {
            Ok(_) => Ok(serde_json::Value::Null),
            Err(meas) => {
                println!(" -> Problems with measurement! {:?}", meas);      // TODO: Add proper error
                Ok(serde_json::Value::Null)
            }
        }
    }

    // /// G29 \
    // /// Return to home position async
    // pub fn g29<R : BasicRobot<C>, const C : usize>
    //     (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    // {
    //     // arm.measure(2);
    //     robot.measure_async(2);
    //     robot.await_inactive();
    //     robot.update(None);
    //     let home = *robot.home_pos();
    //     robot.set_end(&home);
    //     Ok(serde_json::Value::Null)
    // }

    // Extra functions
    pub fn g100<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (_ : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        // let phis = robot.safe_phis(args_by_iterate_fixed::<C>(args, 'A'))?;

        // let deltas = robot.move_j_abs(robot.gammas_from_phis(phis))?;
        // robot.update(None)?;
        // Ok(serde_json::json!({ 
        //     "phis": Vec::from(phis),
        //     "deltas": Vec::from(deltas)
        // }))
        todo!()
    }

    // Debug
    pub fn g1000<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (robot : &mut R, desc : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        Ok(serde_json::json!({ 
            "phis": Vec::from(robot.phis()),
            "gammas": Vec::from(robot.gammas()),
            "pos": desc.current_tcp().pos().to_array()
        }))
    }

    pub fn g1100<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (_ : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        // let phis = robot.safe_phis(args_by_iterate_fixed::<C>(args, 'A'))?;

        // robot.write_phis(&phis);
        // robot.update(None)?;
        // Ok(serde_json::json!(Vec::from(phis)))
        todo!()
    }
//

// Misc Functions
    pub fn m3<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (robot : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        robot.activate_tool()?;
        Ok(serde_json::Value::Null)
    }

    pub fn m4<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (_ : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        // Ok(serde_json::json!(robot.activate_spindle(false)))
        todo!()
    }

    pub fn m5<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (robot : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        robot.deactivate_tool()?;
        todo!()
    }

    pub fn m30<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
    (_ : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        println!("Program finished!");
        exit(0);
    }

    // Additional functions
    pub fn m119<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (_ : &mut R, desc : &mut D, _ : &mut S, _ : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let args = args_by_iterate(args, 'A');
        let mut args_phis = Vec::with_capacity(args.len()); 

        for arg in args {
            args_phis.push(Phi(arg));
        }

        desc.aconf_mut().configure(args_phis)?;
        
        Ok(serde_json::Value::Null)
    }

    // Debug functions
    pub fn m1006<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (robot : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        Ok(serde_json::to_value(robot.get_tool().unwrap().get_json()).unwrap())
        // Ok(serde_json::to_value(
        //     robot.get_tools().iter().map(
        //         |t| t.get_json()
        //     ).collect::<Vec<serde_json::Value>>()
        // ).unwrap())
    }
// 

// Programm functions
    pub fn o0<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
        (_ : &mut R, _ : &mut D, _ : &mut S, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        println!("test");
        Ok(serde_json::Value::Null)
    }
//

// Tool
pub fn t<R : Robot<C>, D : Descriptor<C>, S, const C : usize>
    (_ : &mut R, _ : &mut D, _ : &mut S, _ : usize) -> Result<serde_json::Value, crate::Error> 
{
    // if let Some(tool) = robot.set_tool_id(index) {
    //     return Ok(tool.get_json())
    // }

    Ok(serde_json::Value::Null)
    // Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, " -> No tool has been found for this index!"))
}
