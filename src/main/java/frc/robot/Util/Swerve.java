package frc.robot.Util;



//PID
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;

//SENSORS
import edu.wpi.first.math.kinematics.SwerveModuleState;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;

public class Swerve {
  public TalonFX driveMotor;
  private TalonFX angleMotor;
  private PIDController anglePID;
  private PIDController drivePID;
  private CANCoder angleEncoder;



    public Swerve(int motorIdDrive,int motorIdAngle,int encoderID, PIDController pidA, PIDController pidD, double offSet){
        angleEncoder = new CANCoder(encoderID);
        angleEncoder.configMagnetOffset(offSet);
        drivePID = pidA;
        anglePID = pidD;
        driveMotor = new TalonFX(motorIdDrive);
        angleMotor = new TalonFX(motorIdAngle);
        driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        angleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        angleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
        
    }

    public void resetDriveEnc(){
        driveMotor.setSelectedSensorPosition(0);
    }

    public double totalDis(){
        return driveMotor.getSelectedSensorPosition();
    }

    public void setSpeed(double driveSpeed, double angleSpeed){
        driveMotor.set(ControlMode.PercentOutput, driveSpeed);
        angleMotor.set(ControlMode.PercentOutput, angleSpeed);
    }

    public void setAngleSpeed(double angleSpeed){
        angleMotor.set(ControlMode.PercentOutput, angleSpeed);
    }

    public void setDriveSpeed(double driveSpeed){
        driveMotor.set(ControlMode.PercentOutput, driveSpeed);
    }
    public double drivePIDCalc(double rate){
        double calcDriveSpeed = 0;
        calcDriveSpeed = drivePID.calculate(getMetersPerSec(), rate);
        return calcDriveSpeed;
    }

    public double angleOffsetAnglePID(double angle, double offSet){
        return anglePIDCalcABS(angle);
    }

    public double angleOffsetDrivePID(double angle, double offSet){
        return anglePIDCalcABS(angle);
    }

    public double anglePIDCalc(double angle){
        double calcAngleSpeed = 0;
        anglePID.calculate(angleEncoder.getPosition(), angle);
        return calcAngleSpeed;
    }

    public double anglePIDCalcABS(double angle){
        double calcAngleSpeed = 0;
        
        if(angle != 42069) {
            double curPos = angleEncoder.getAbsolutePosition();
            double posAngle = angle + 360;
            double negAngle = angle - 360;
            
            double curError = Math.abs(curPos - angle);
            double posError = Math.abs(curPos - posAngle);
            double negError = Math.abs(curPos - negAngle);

            if(curError < posError && curError < negError) {
                calcAngleSpeed = anglePID.calculate(angleEncoder.getAbsolutePosition(), angle);
            } else if(posError < curError && posError < negError) {
                calcAngleSpeed = anglePID.calculate(angleEncoder.getAbsolutePosition(), posAngle);
            } else {
                calcAngleSpeed = anglePID.calculate(angleEncoder.getAbsolutePosition(), negAngle);
            }
        }
        
        return calcAngleSpeed;
    }

    public void resetEncoder(){
        angleEncoder.setPosition(0);
    }

    public double getEncoder(){
        return angleEncoder.getAbsolutePosition();
    }
    public double getDriveEncoder(){
        return driveMotor.getSelectedSensorPosition();
    }
    public void setDriveModeBrake(){
        driveMotor.setNeutralMode(NeutralMode.Brake);
    }
    public void setDriveModeCoast(){
        driveMotor.setNeutralMode(NeutralMode.Coast);
    }
    //new functions
    public double getMetersPerSec() {
        return driveMotor.getSelectedSensorVelocity() * Constants.velocityToMeters;
    }
    public void setMetersPerSec(double metersPerSec) {
        //some way to go the correct
        double FF = 0.25735*(metersPerSec)-0.01786;
        double pidVal = drivePIDCalc(getMetersPerSec());
        setDriveSpeed(FF + pidVal);
    }
    public void modState(SwerveModuleState state) {
        state.angle.getDegrees();
        setAngleSpeed(anglePIDCalcABS(state.angle.getDegrees()+90));
        setMetersPerSec(state.speedMetersPerSecond);
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(getMetersPerSec(), new Rotation2d(Math.toRadians(angleEncoder.getAbsolutePosition()-90)));
    }
}