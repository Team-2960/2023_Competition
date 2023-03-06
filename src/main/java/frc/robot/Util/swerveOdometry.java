package frc.robot.Util;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public class swerveOdometry{
    
    
    public swerveOdometry(){
        
    }

    public static swerveOdometry get_Instance(){
        return null;
    }

    public void updateOdometry(SwerveModuleState fl, SwerveModuleState fr, SwerveModuleState bl, SwerveModuleState br, double gyroHeadingRad, double timeStep){
        
    }


    public static  Pose2d getRobotPose(){
        return new Pose2d(0, 0, new Rotation2d(0));
    }
}
