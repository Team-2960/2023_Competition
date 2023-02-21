package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;

public class Drive {
    //VARS
    public static Drive drive; 

    //MOTOR VARS
    private TalonFX BLAngle;
    private TalonFX BLDrive;
    private TalonFX FLAngle;
    private TalonFX FLDrive;
    private TalonFX FRAngle;
    private TalonFX FRDrive;
    private TalonFX BRAngle;
    private TalonFX BRDrive;

    //ENCODER VARS
    private CANCoder BLEnc;
    private CANCoder FLEnc;
    private CANCoder FREnc;
    private CANCoder BREnc;



    public static Drive get_Instance(){
        if(drive == null){
            drive = new Drive();
        }
        return drive;
    }

    private Drive(){
        BLAngle = new TalonFX(Constants.BLAngle);
        BLDrive = new TalonFX(Constants.BLDrive);
        FLAngle =  new TalonFX(Constants.FLAngle);
        FLDrive =  new TalonFX(Constants.FLDrive);
        FRAngle =  new TalonFX(Constants.FRAngle);
        FRDrive =  new TalonFX(Constants.FRDrive);
        BRAngle =  new TalonFX(Constants.BRAngle);
        BRDrive =  new TalonFX(Constants.BRDrive);
        BLEnc = new CANCoder(Constants.BLEnc);
        FLEnc = new CANCoder(Constants.FLEnc);
        FREnc = new CANCoder(Constants.FREnc);
        BREnc = new CANCoder(Constants.BREnc);
    }

    public void setBLAngle(double pctSpeed){
        BLAngle.set(TalonFXControlMode.PercentOutput, pctSpeed); 
    
    }

    public void setBLDrive(double pctSpeed){
        BLDrive.set(TalonFXControlMode.PercentOutput, pctSpeed);
    }
    
    public void setFLAngle(double pctSpeed){
        FLAngle.set(TalonFXControlMode.PercentOutput, pctSpeed);
    }

    public void setFLDrive(double pctSpeed){
        FLDrive.set(TalonFXControlMode.PercentOutput, pctSpeed);
    }

    public void setFRAngle(double pctSpeed){
        FRAngle.set(TalonFXControlMode.PercentOutput, pctSpeed);
    }
    
    public void setFRDrive(double pctSpeed){
        FRDrive.set(TalonFXControlMode.PercentOutput, pctSpeed);
    }

    public void setBRAngle(double pctSpeed){
    BRAngle.set(TalonFXControlMode.PercentOutput, pctSpeed);
    }

    public void setBRDrive(double pctSpeed){
        BRDrive.set(TalonFXControlMode.PercentOutput, pctSpeed);
    }

    public void printCANCoderBL(){
        System.out.println(BLEnc.getAbsolutePosition());
    }
    
    public void printCANCoderFL(){
        System.out.println(FLEnc.getAbsolutePosition());
    }

    public void printCANCoderBR(){
        System.out.println(BREnc.getAbsolutePosition());
    }

    public void printCANCoderFR(){
        System.out.println(FREnc.getAbsolutePosition());
    }
}

    