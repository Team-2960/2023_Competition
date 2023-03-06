package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorClaw;
import frc.robot.subsystems.Intake;

public class OI {
    // INSTANCE
    private static OI oi;

    // THE SUBSYSTEMS
    private static Drive drive;
    private static ElevatorClaw elevatorClaw;
    private static Intake intake;

    // JOYSTICKS
    private static Joystick driverControl;
    private static Joystick operatorControl;
    private static Joystick testJoy1;
    private static Joystick testJoy2;

    private OI() {
        // Instantiate the subsystems
        drive = Drive.get_Instance();
        elevatorClaw = ElevatorClaw.get_Instance();
        intake = Intake.get_Instance();
        // Create Joysticks
        driverControl = new Joystick(Constants.driverControlID);
        operatorControl = new Joystick(Constants.operatorControlID);
        testJoy1 = new Joystick(Constants.testJoy1);
        testJoy2 = new Joystick(Constants.testJoy2);

    }

    public static OI get_Instance() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }

    public void testOI() {

        // BL

        if (testJoy1.getRawButton(1)) {
            drive.setBLDrive(0.2);
        } else {
            drive.setBLDrive(0);
        }
        // FL
        if (testJoy1.getRawButton(2)) {
            drive.setFLAngle(0.2);
        } else {
            drive.setFLAngle(0);
        }
        if (testJoy1.getRawButton(3)) {
            drive.setFLDrive(0.2);
        } else {

            drive.setFLDrive(0);
        }
        // FR
        if (testJoy1.getRawButton(4)) {
            drive.setFRAngle(0.2);
        } else {
            drive.setFRAngle(0);
        }
        if (testJoy1.getRawButton(5)) {
            drive.setFRDrive(0.2);
        } else {
            drive.setFRDrive(0);
        }
        // BR
        if (testJoy1.getRawButton(6)) {
            drive.setBRAngle(0.2);
        } else {
            drive.setBRAngle(0);
        }
        if (testJoy1.getRawButton(7)) {
            drive.setBRDrive(0.2);
        } else {
            drive.setBRDrive(0);
        }

        // PRINT CANCODERS
        /*
         * drive.printCANCoderBL();
         * drive.printCANCoderFL();
         * drive.printCANCoderBR();
         * drive.printCANCoderFR();
         */
        System.out.println(intake.getGamePiecePhotoeye());

        // Elevator Claw
        /*
         * if(testJoy1.getRawButton(1)){
         * elevatorClaw.setElevator(0.2, 0);
         * }
         * else if(testJoy1.getRawButton(2)){
         * elevatorClaw.setElevator(0, 0.2);
         * }else{
         * elevatorClaw.setElevator(0, 0);
         * }
         */ if (testJoy1.getRawButton(3)) {
            elevatorClaw.setGripperState(Value.kForward);
        } else if (testJoy1.getRawButton(4)) {
            elevatorClaw.setGripperState(Value.kReverse);
        }
        // TEST JOY 2

        if (testJoy2.getRawButton(5)) {
            elevatorClaw.setWristState(Value.kForward);
        } else if (testJoy2.getRawButton(6)) {
            elevatorClaw.setWristState(Value.kReverse);
        }

        // INTAKE
        if (testJoy2.getRawButton(4)) {
            // intake.setConveyorSpeed(0.2);
            intake.setIntakeForward(true);
        } else {
            // intake.setConveyorSpeed(0);
            intake.setIntakeForward(false);

        }
        /*
         * if(testJoy2.getRawButton(5)){
         * intake.setFlappySpeed(1);
         * }else{
         * intake.setFlappySpeed(0);
         * }
         * if(testJoy2.getRawButton(6)){
         * intake.setIntakeSpeed(-1);
         * }else{
         * intake.setIntakeSpeed(0);
         * }
         */
        if (testJoy2.getRawButton(7)) {
            intake.setIntakeState(Value.kForward);
        } else if (testJoy2.getRawButton(8)) {
            intake.setIntakeState(Value.kReverse);
        }
        if (testJoy2.getRawButton(9)) {
            elevatorClaw.setStopperState(Value.kForward);
        } else if (testJoy2.getRawButton(10)) {
            elevatorClaw.setStopperState(Value.kReverse);
        }
    }

    public void soberOI() {

           // PRINT CANCODERS
        
         drive.printCANCoderBL();
         drive.printCANCoderFL();
         drive.printCANCoderBR();
         drive.printCANCoderFR();
        
        // Back button is close claw, start button is open claw
        if (testJoy1.getRawButton(7)) {
            elevatorClaw.setGripperState(Value.kForward);
        }
        if (testJoy1.getRawButton(8)) {
            elevatorClaw.setGripperState(Value.kReverse);
        }

        // X button brings wrist down, Y button bings wrist up
        if (testJoy1.getRawButton(3)) {
            elevatorClaw.setWristState(Value.kForward);
        }
        if (testJoy1.getRawButton(4)) {
            elevatorClaw.setWristState(Value.kReverse);
        }

        // A button is stopper down, B button is stopper up
        if (testJoy1.getRawButton(1)) {
            elevatorClaw.setStopperState(Value.kForward);
        }
        if (testJoy1.getRawButton(2)) {
            elevatorClaw.setStopperState(Value.kReverse);
        }

        if (testJoy2.getRawButton(3)) {
            intake.setIntakeState(Value.kForward);
        }
        if (testJoy2.getRawButton(4)) {
            intake.setIntakeState(Value.kReverse);
        }

        if (testJoy2.getRawButton(5)){
            intake.setIntakeForward(true);
        }
        else {
            intake.setIntakeForward(false);
        }
    }

}
