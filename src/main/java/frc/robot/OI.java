package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorClaw;
import frc.robot.subsystems.Intake;

public class OI {
    //INSTANCE
    private static OI oi;

    //THE SUBSYSTEMS
    private static Drive drive;
    private static ElevatorClaw elevatorClaw;
    private static Intake intake;

    //JOYSTICKS
    private static Joystick driverControl;
    private static Joystick operatorControl;
    private static Joystick testJoy1;
    private static Joystick testJoy2;

    private OI(){
        //Instantiate the subsystems
        drive = Drive.get_Instance();
        elevatorClaw = ElevatorClaw.get_Instance();
        intake = Intake.get_Instance();
        //Create Joysticks
        driverControl = new Joystick(Constants.driverControlID);
        operatorControl = new Joystick(Constants.operatorControlID);
        testJoy1 = new Joystick(Constants.testJoy1);
        testJoy2 = new Joystick(Constants.testJoy2);

    }

    public static OI get_Instance(){
        if(oi == null){
            oi = new OI();
        }
        return oi;
    }
    
    public void testOI(){
        //BL
        if(testJoy1.getRawButton(0)){
            drive.setBLAngle(0.2);
        }else{
            drive.setBLAngle(0);
        }
        if(testJoy1.getRawButton(1)){
            drive.setBLDrive(0.2);
        }else{
            drive.setBLDrive(0);
        }
        //FL
        if(testJoy1.getRawButton(2)){
            drive.setFLAngle(0.2);
        }else{
            drive.setFLAngle(0);
        }
        if(testJoy1.getRawButton(3)){
            drive.setFLDrive(0.2);
        }else{
            drive.setFLDrive(0);
        }
        //FR
        if(testJoy1.getRawButton(4)){
            drive.setFRAngle(0.2);
        }else{
            drive.setFRAngle(0);
        }
        if(testJoy1.getRawButton(5)){
            drive.setFRDrive(0.2);
        }else{
            drive.setFRDrive(0);
        }
        //BR
        if(testJoy1.getRawButton(6)){
            drive.setBRAngle(0.2);
        }else{
            drive.setBRAngle(0);
        }
        if(testJoy1.getRawButton(7)){
            drive.setBRDrive(0.2);
        }else{
            drive.setBRDrive(0);
        }

        //PRINT CANCODERS
        drive.printCANCoderBL();
        drive.printCANCoderFL();
        drive.printCANCoderBR();
        drive.printCANCoderFR();

        //Elevator Claw
        if(testJoy1.getRawButton(8)){
            elevatorClaw.setElevator(0.2, 0);
        }else{
            elevatorClaw.setElevator(0, 0);
        }
        if(testJoy1.getRawButton(9)){
            elevatorClaw.setElevator(0, 0.2);
        }else{
            elevatorClaw.setElevator(0, 0);
        }
        if(testJoy1.getRawButton(10)){
            elevatorClaw.setGripperState(Value.kForward);
        }else if(testJoy1.getRawButton(11)){
            elevatorClaw.setGripperState(Value.kReverse);
        }
        //TEST JOY 2
        
        if(testJoy2.getRawButton(2)){
            elevatorClaw.setWristState(Value.kForward);
        }else if(testJoy2.getRawButton(3)){
            elevatorClaw.setWristState(Value.kReverse);
        }

        //INTAKE
        if(testJoy2.getRawButton(4)){
            intake.setConveyorSpeed(0.2);
        }else{
            intake.setConveyorSpeed(0);
        }
        if(testJoy2.getRawButton(5)){
            intake.setFlappySpeed(0.2);
        }else{
            intake.setFlappySpeed(0);
        }
        if(testJoy2.getRawButton(6)){
            intake.setIntakeSpeed(0.2);
        }else{
            intake.setIntakeSpeed(0);
        }
        if(testJoy2.getRawButton(7)){
            intake.setIntakeState(Value.kForward);
        }else if(testJoy2.getRawButton(8)){
            intake.setIntakeState(Value.kReverse);
        }
        if(testJoy2.getRawButton(9)){
            elevatorClaw.setStopperState(Value.kForward);
        }else if(testJoy2.getRawButton(10)){
            elevatorClaw.setStopperState(Value.kReverse);
        }
    }
}
