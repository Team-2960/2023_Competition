package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Util.Swerve;
import frc.robot.Util.swerveOdometry;

public class Drive {
    //VARS
    public static Drive drive; 

    //MOTOR VARS
    private static TalonFX BLAngle;
    private static TalonFX BLDrive;
    private static TalonFX FLAngle;
    private static TalonFX FLDrive;
    private static TalonFX FRAngle;
    private static TalonFX FRDrive;
    private static TalonFX BRAngle;
    private static TalonFX BRDrive;

    //ENCODER VARS
    private CANCoder BLEnc;
    private CANCoder FLEnc;
    private CANCoder FREnc;
    private CANCoder BREnc;

    //Swerve Vars
    public Swerve frontLeft;
    public Swerve frontRight;
    public Swerve backRight;
    public Swerve backLeft;

    //Swerve PID VARS
    private static PIDController PIDDFL;
    private static PIDController PIDAFL;
    private static PIDController PIDDFR;
    private static PIDController PIDAFR;
    private static PIDController PIDDBL;
    private static PIDController PIDABL;
    private static PIDController PIDDBR;
    private static PIDController PIDABR;

    //Swerve States
    private static double frontLeftSwerveSpeed;
    private static double frontLeftSwerveAngle;
    private static double frontRightSwerveSpeed;
    private static double frontRightSwerveAngle;
    private static double backLeftSwerveSpeed;
    private static double backLeftSwerveAngle;
    private static double backRightSwerveSpeed;
    private static double backRightSwerveAngle;

    private double angleRateVector;

    //Sensor Vars
    private static AHRS navX;
    private double gyroAngle;

    //Autonomous Variables

    //Odometry Vars
    public swerveOdometry so;
    private double prevTime;
    private Timer autoTimer;

    // Swerve drive auton
    private SwerveDriveKinematics m_kinematics;
    public double velX;
    public double velY;
    public double omega;
    public Pose2d swervePose;
    private Translation2d m_frontLeftLocation;
    private Translation2d m_frontRightLocation;
    private Translation2d m_backLeftLocation;
    private Translation2d m_backRightLocation;

    public static Drive get_Instance(){
        if(drive == null){
            drive = new Drive();
        }
        return drive;
    }

    //Init Functions
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

        //NavX
        navX = new AHRS(SPI.Port.kMXP, (byte) 200);
        navX.calibrate();
        navX.resetDisplacement();

        //Construct PID's for Swerve
        PIDDFL = new PIDController(Constants.dPFL, Constants.dIFL, Constants.dDFL);
        PIDAFL = new PIDController(Constants.aPFL, Constants.aIFL, Constants.aDFL);
        PIDDFR = new PIDController(Constants.dPFR, Constants.dIFR, Constants.dDFR);
        PIDAFR = new PIDController(Constants.aPFR, Constants.aIFR, Constants.aDFR);
        PIDDBL = new PIDController(Constants.dPBL, Constants.dIBL, Constants.dDBL);
        PIDABL = new PIDController(Constants.aPBL, Constants.aIBL, Constants.aDBL);
        PIDDBR = new PIDController(Constants.dPBR, Constants.dIBR, Constants.dDBR);
        PIDABR = new PIDController(Constants.aPBR, Constants.aIBR, Constants.aDBR);

        //Create Swerve
        frontLeft = new Swerve(Constants.FLDrive, Constants.FLAngle,
            Constants.FLEnc, PIDDFL, PIDAFL, Constants.flHome);
        frontRight = new Swerve(Constants.FRDrive, Constants.FRAngle,
            Constants.FREnc, PIDDFR, PIDAFR, Constants.frHome);
        backRight = new Swerve(Constants.BRDrive, Constants.BRAngle,
            Constants.BREnc, PIDDBR, PIDABR, Constants.brHome);
        backLeft = new Swerve(Constants.BLDrive, Constants.BLAngle,
            Constants.BLEnc, PIDDBL, PIDABL, Constants.blHome);

        //RESET The Drive Encoders
        frontLeft.resetDriveEnc();
        frontRight.resetDriveEnc();
        backLeft.resetDriveEnc();
        backRight.resetDriveEnc();

        //Construct Kinematics
        m_frontLeftLocation = new Translation2d(-Constants.modLoc, -Constants.modLoc);
        m_frontRightLocation = new Translation2d(Constants.modLoc, -Constants.modLoc);
        m_backLeftLocation = new Translation2d(-Constants.modLoc, Constants.modLoc);
        m_backRightLocation = new Translation2d(Constants.modLoc, Constants.modLoc);

        m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        //Construct Odometry
        so = swerveOdometry.get_Instance();

        autoTimer = new Timer();
    }

    //Set the motors to break mode
    public void breakMode() {
        frontRight.setDriveModeBrake();
        frontLeft.setDriveModeBrake();
        backRight.setDriveModeBrake();
        backLeft.setDriveModeBrake();
    }

    //Set the motors to coast mode
    public void coastMode() {
        frontRight.setDriveModeCoast();
        frontLeft.setDriveModeCoast();
        backRight.setDriveModeCoast();
        backLeft.setDriveModeCoast();
    }

    //Teleop Functions
    //Set the adjustment offset for the NavX
    public static void setNavX(double offset){
        navX.setAngleAdjustment(offset);
    }

    //Get the Robot Pose
    public Pose2d getRobotPos(){
        return swerveOdometry.getRobotPose();
    }

    //Update the swerve drive odometry
    public void updateOdometry(){
        autoTimer.start();
        double currTime = autoTimer.get();
        double timeStep = currTime - prevTime;
        prevTime = currTime;
        
        // Front left module state
        SwerveModuleState fl = frontLeft.getState();
        // Front right module state
        SwerveModuleState fr = frontRight.getState();
        // Back left module state
        SwerveModuleState bl = backLeft.getState();
        // Back right module state
        SwerveModuleState br = backRight.getState();
        so.updateOdometry(fl, fr, bl, br, navX.getAngle(), timeStep);
    }

    //Set the swerve vectors (Rotation and Velocity)
    public void setSwerve(double angleVectorX, double angleVectorY, double rotationVectorX) {
        boolean isDeadZone = Math.abs(angleVectorX) < .15 &&
            Math.abs(angleVectorY) < .15 &&
            Math.abs(rotationVectorX) < 1;
        if (!isDeadZone) {
            double rotationVectorY = rotationVectorX;
            double A = angleVectorX + rotationVectorX;// THE PLUS AND MINUS MAY BE FLIPPED
            double B = angleVectorX - rotationVectorX;
            double C = angleVectorY + rotationVectorY;
            double D = angleVectorY - rotationVectorY;
            frontLeftSwerveSpeed = Math.sqrt(Math.pow(A, 2.0) + Math.pow(C, 2.0));
            frontLeftSwerveAngle = 180;//Math.atan2(A, C) * 180 / Math.PI;
            backLeftSwerveSpeed = Math.sqrt(Math.pow(A, 2.0) + Math.pow(D, 2.0));
            backLeftSwerveAngle = Math.atan2(A, D) * 180 / Math.PI;
            frontRightSwerveSpeed = Math.sqrt(Math.pow(B, 2.0) + Math.pow(C, 2.0));
            frontRightSwerveAngle = Math.atan2(B, C) * 180 / Math.PI;
            backRightSwerveSpeed = Math.sqrt(Math.pow(B, 2.0) + Math.pow(D, 2.0));
            backRightSwerveAngle = Math.atan2(B, D) * 180 / Math.PI;
        } else {
            frontLeftSwerveSpeed = 0;
            frontRightSwerveSpeed = 0;
            backLeftSwerveSpeed = 0;
            backLeftSwerveSpeed = 0;
        }
    }

    //WHAT GETS CALLED BY OI!!!
    public void setVector(double angle, double mag, double rotationVectorX) {
        double angleVX = Math.cos((angle - gyroAngle) * Math.PI / 180) * 180 / Math.PI * mag;
        double angleVY = Math.sin((angle - gyroAngle) * Math.PI / 180) * 180 / Math.PI * mag;

        //No rate PID Implemented Yet
        angleRateVector = -15 * rotationVectorX;//targetAngleRate = rotationVectorX;
        
        if(Math.abs(angleRateVector) < 3 && Math.abs(navX.getRate()) > 0.02){
          angleRateVector = 25 * navX.getRate();
        }
        setSwerve(angleVX, angleVY, angleRateVector);
    }

    public void sanitizeAngle() {
        gyroAngle = navX.getAngle();
        while (gyroAngle > 360) {
          gyroAngle = gyroAngle - 360;
        }
        while (gyroAngle < 0) {
          gyroAngle = gyroAngle + 360;
        }    
    }

    //Called instead of the periodic function
    public void periodicTele() {
        SmartDashboard.putNumber("fl Angle", frontLeft.getEncoder());
        SmartDashboard.putNumber("fr Angle", frontRight.getEncoder());
        SmartDashboard.putNumber("bl Angle", backLeft.getEncoder());
        SmartDashboard.putNumber("br Angle", backRight.getEncoder());

        SmartDashboard.putNumber("fl delta", frontLeftSwerveAngle - frontLeft.getEncoder());
        SmartDashboard.putNumber("error", frontLeft.anglePID.getPositionError());
        SmartDashboard.putNumber("tar pos", frontLeftSwerveAngle);
        SmartDashboard.putNumber("fl speed", frontLeft.anglePIDCalcABS(frontLeftSwerveAngle));
        //updateOdometry();
        sanitizeAngle();
        frontLeft.setSpeed(frontLeftSwerveSpeed/75, frontLeft.anglePIDCalcABS(frontLeftSwerveAngle));
        frontRight.setSpeed(frontRightSwerveSpeed/75, frontRight.anglePIDCalcABS(frontRightSwerveAngle));
        backLeft.setSpeed(backLeftSwerveSpeed/75, backLeft.anglePIDCalcABS(backLeftSwerveAngle));
        backRight.setSpeed(backRightSwerveSpeed/75, backRight.anglePIDCalcABS(backRightSwerveAngle));
    }

    //Autonomous Functions

    //TODO Set the Autonomous Functions to use this function
    //Set the vars that control the autonomous movement
    public void setAutonomousSpeeds(double x, double y, double omega) {
        velY = y;
        velX = x;
        this.omega = omega;
    }

    //Update the speeds for autnomous movement
    public void autonUpdate() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velY, velX, omega, Rotation2d.fromDegrees(-navX.getYaw()));
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    
        // Front left module state
        SwerveModuleState backRightState = moduleStates[0];
    
        // Front right module state
        SwerveModuleState frontRightState = moduleStates[1];
    
        // Back left module state
        SwerveModuleState backLeftState = moduleStates[2];
    
        // Back right module state
        SwerveModuleState frontLeftState = moduleStates[3];
          
        frontRight.modState(frontRightState);
        frontLeft.modState(frontLeftState);
        backRight.modState(backRightState);
        backLeft.modState(backLeftState);
    }

    //Test Functions
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
        SmartDashboard.putNumber("BLEnc",BLEnc.getAbsolutePosition());
    }
    
    public void printCANCoderFL(){
        SmartDashboard.putNumber("FLEnc",FLEnc.getAbsolutePosition());
    }

    public void printCANCoderBR(){
        SmartDashboard.putNumber("BREnc",BREnc.getAbsolutePosition());
    }

    public void printCANCoderFR(){
        SmartDashboard.putNumber("FREnc",FREnc.getAbsolutePosition());
    }
}

    