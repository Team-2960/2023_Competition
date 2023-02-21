package frc.robot;

public class Constants { 
    //JOYSTICKS
    public final static int driverControlID = 0;
    public final static int operatorControlID = 1;
    public final static int testJoy1 = 2;
    public final static int testJoy2 = 3;

    //SWERVE-AUTON
    public final static double velocityToMeters = (10/8.16*Math.PI*3.9*0.0254)/(2048);

    //PhotoEyes
    public final static int gamePiecePhotoeye = 0;
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
    public final static int mLElevator =14;
    public final static int mFlapperWheels = 15;
    public final static int mIntakeWheels = 16;
    public final static int mConveyor = 17;

    //SOLENOID
    public final static int[] sGripperID = {4,5};
    public final static int[] sIntakeID = {2,3};
    public final static int[] sWristID = {6,7};
    public final static int[] sStopperID = {0,1};

    //MISC CAN
    public final static int PneumHub = 18;
    public final static int PDH = 19;
}
