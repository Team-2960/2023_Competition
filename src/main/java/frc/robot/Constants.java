package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants { 
    //JOYSTICKS
    public final static int driverControlID = 0;
    public final static int operatorControlID = 1;
    public final static int testJoy1 = 2;
    public final static int testJoy2 = 3;
    public final static int manualControl = 4;

    //SWERVE-AUTON
    public final static int ticksPerRev = 2048;
    public final static double metersPerInch = 0.0254;
    public final static double wheelDiameter = Math.PI*3.9 * metersPerInch;
    public final static double gearReduction = 8.14;

    public final static double upWristPos = 0.47;//42!!
    public final static double level2Wrist = 0.3;

    public final static double velocityToMeters = (10*(1/gearReduction)*wheelDiameter)/(ticksPerRev);
    public final static double positionToMeters = (1/gearReduction*wheelDiameter)/(ticksPerRev);



    //Theta Thresholding
    public static double thresholdT1 = 0.075;
    public static double tVel1 = 0;

    public static double thresholdT2 = 0.2;
    public static double tVel2 = 0.7;

    public static double thresholdT3 = 0.7;
    public static double tVel3 = 2.1;

    public static double tVelOutside = 2.1;

    //PhotoEyes
    public final static int gamePiecePhotoeye = 3;
    public final static int wristEncoder0 = 0;
    public final static int LowerElevatorPhotoeye = 1;
    public final static int UpperElevatorPhotoeye = 2;

    //SWERVE
    public final static int BLAngle = 1;
    public final static int BLDrive = 2;
    public final static int FLAngle = 3;
    public final static int FLDrive = 4;
    public final static int FRAngle = 5; 
    public final static int FRDrive = 6; 
    public final static int BRAngle = 7;
    public final static int BRDrive = 8;
    public final static int BLEnc = 9;
    public final static int FLEnc = 10;
    public final static int FREnc = 11;
    public final static int BREnc = 12;

    //MOTORS
    public final static int mRElevator = 13;
    //public final static int mLElevator =14;
    public final static int mFlapperWheels = 15;
    public final static int mIntakeWheels = 16;
    public final static int mConveyor = 17;

    //SOLENOID
    public final static int[] sGripperID = {9,5}; //4,5
    public final static int[] sIntakeID = {2,3};
    public final static int[] sWristID = {7,6};
    public final static int[] sStopperID = {15,14};
    public final static int sPusherID = 4;


    //MISC CAN
    public final static int PneumHub = 18;
    public final static int PDH = 19;

    //Elevator PID
    public final static double kElevatorp = .00006;
    public final static double kElevatori = 0;
    public final static double kElevatord = 0;

    //Elevator Speed PID
    public final static double elvSpeedP = 0.00002;//0.000017;
    public final static double elvSpeedI= 0.0;//0.000003;
    public final static double elvSpeedD = 0.0;//0.000002;

    //Elevator Level Positions
    public final static double cHome = -100;
    public final static double cLevel1 = 27000;
    public final static double cLevel2 = 22000;
    public final static double cLevel3 = 41000;
    public final static double cFeeder = 5000;
    //elevator Limits
    public final static double elevatorMaxPos = 45000;
    public final static double elevatorMinPos = -3000;
    public final static double moveWristLimit = 38000;

    //PID Constants Swerve
    public static double dPFL = 0.0008;
    public static double dIFL = 0;
    public static double dDFL = 0;
    public static double aPFL = 0.008;
    public static double aIFL = 0.0004;
    public static double aDFL = 0.00;
    public static double dPFR = 0.0008;
    public static double dIFR = 0;
    public static double dDFR = 0;
    public static double aPFR = 0.008;
    public static double aIFR = 0.0004;
    public static double aDFR = 0.00;
    public static double dPBL = 0.0008;
    public static double dIBL = 0;
    public static double dDBL = 0;
    public static double aPBL = 0.008;
    public static double aIBL = 0.0004;
    public static double aDBL = 0.00;
    public static double dPBR = 0.0008;//0.0008
    public static double dIBR = 0;
    public static double dDBR = 0;
    public static double aPBR = 0.008;//0.007
    public static double aIBR = 0.0004;
    public static double aDBR = 0.00;

    //Swerve Home Constants
    public static double flHome = -324.58+90;//-7.27;
    public static double frHome = -275.8+90;//-269.19;
    public static double blHome = -203.64+90;//-73.81;
    public static double brHome = -176.74-180+90;//-157.03;
    public static double modLoc = 12  * 0.0254;
    //Swerve Drive Kinematics


}
