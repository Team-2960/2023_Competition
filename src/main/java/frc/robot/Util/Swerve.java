package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//MOTORS
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//PID
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//SENSORS
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class Swerve {
  public TalonFX driveMotor;
  private TalonFX angleMotor;
  public PIDController anglePID;
  private PIDController drivePID;
  private CANCoder angleEncoder;
  private Translation2d translation2d;
  private Rotation2d rotation2d;
  private Pose2d pose2d;

  private double prevEncPos = 0;
  private double prevTime = 0;

  private Timer timer;
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
        
        timer = new Timer();
        timer.start();
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
        //angle *= 180/Math.PI;
        double calcAngleSpeed = 0;

        angle = Drive.reduceAngle(angle);
        
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
        
        return -calcAngleSpeed;
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
        double dTime = (timer.get() - prevTime);
        double dTheta = driveMotor.getSelectedSensorPosition() - prevEncPos;
        prevEncPos = driveMotor.getSelectedSensorPosition();
        prevTime = timer.get();
        //return ((Math.PI*3.9*0.0254)/(2048*8.16)) * (dTheta/dTime);
        return driveMotor.getSelectedSensorVelocity() * Constants.velocityToMeters;
    }
    public void setMetersPerSec(double metersPerSec) {
        //some way to go the correct
        double FF = 0.25735*(metersPerSec)-0.01786;
        double pidVal = drivePIDCalc(getMetersPerSec());
        setDriveSpeed(FF + pidVal);
    }
    public void modState(SwerveModuleState state) {
        double curAngle = angleEncoder.getAbsolutePosition();
        double posAngle = curAngle + 180;
        double tarAngle = state.angle.getDegrees() + 90;


        double curError = Math.abs(tarAngle - curAngle);
        double posError = Math.abs(tarAngle - posAngle);

        //Goes to the "positive" error if it is less than the current error
        if(posError < curError){
            setAngleSpeed(anglePIDCalcABS(tarAngle-180));
            setMetersPerSec(-state.speedMetersPerSecond);
        }else{
            setAngleSpeed(anglePIDCalcABS(tarAngle));
            setMetersPerSec(state.speedMetersPerSecond);
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getMetersPerSec(), new Rotation2d(Math.toRadians(angleEncoder.getAbsolutePosition()-90)));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveEncoder() * Constants.positionToMeters, new Rotation2d(Math.toRadians(angleEncoder.getAbsolutePosition()-90)));
    }

    public String toString(){
        String str = "Angle: " + getEncoder() + "\n" + 
                     "M/S: " + getMetersPerSec();
        return str;
    }
}

