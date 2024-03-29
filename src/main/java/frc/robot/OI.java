package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.Joystick;
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
    private static Joystick manualControl;

    //Manual Variables
    private static boolean isManualWrist;

    private OI() {
        // Instantiate the subsystems
        drive = Drive.get_Instance();
        elevatorClaw = ElevatorClaw.get_Instance();
        intake = Intake.get_Instance();
        isManualWrist = false;
        // Create Joysticks
        driverControl = new Joystick(Constants.driverControlID);
        operatorControl = new Joystick(Constants.operatorControlID);
        testJoy1 = new Joystick(Constants.testJoy1);
        testJoy2 = new Joystick(Constants.testJoy2);
        manualControl = new Joystick(Constants.manualControl);
    }

    public static OI get_Instance() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }

    public double driveAngle(double x, double y) {
        double done = 0;
        y = -y;
        if (x < 0) {
            if (y > 0) {
                done = (180 / Math.PI) * (3 * Math.PI / 2 - Math.atan(y / -x));
            } else {
                done = (180 / Math.PI) * (3 * Math.PI / 2 + Math.atan(-y / -x));
            }
        } else if (x > 0) {
            if (y > 0) {
                done = (180 / Math.PI) * (Math.PI / 2 + Math.atan(y / x));
            } else {
                done = (180 / Math.PI) * (Math.PI / 2 - Math.atan(-y / x));
            }
        } else {
            if (y < 0) {
                done = 1;
            } else {
                done = 180;
            }

        }

        done = Math.abs(done - 360) -180;
        return done;
    }

    public void regularOI(){
        //Drive Code
        if(driverControl.getRawButton(1)){
            drive.isXWheels = true;
        }else if(driverControl.getRawAxis(3) > 0.2){
            drive.isXWheels = false;
            double amp = driverControl.getRawAxis(3) + 1;
            drive.setVector(driveAngle(driverControl.getRawAxis(0), driverControl.getRawAxis(1)),
            Math.sqrt(Math.pow(Math.abs(driverControl.getRawAxis(0) * amp), 2)
                    + Math.pow(Math.abs(driverControl.getRawAxis(1) * amp), 2)),
                 driverControl.getRawAxis(4) * -2);
        }else{
            drive.isXWheels = false;
            drive.setVector(driveAngle(driverControl.getRawAxis(0), driverControl.getRawAxis(1)),
                        Math.sqrt(Math.pow(Math.abs(driverControl.getRawAxis(0)), 2)
                                + Math.pow(Math.abs(driverControl.getRawAxis(1)), 2)),
                             driverControl.getRawAxis(4) * -2);
        }
        //Elevator Control
       if (operatorControl.getRawButton(1)){
            elevatorClaw.setElevatorState(ElevatorClaw.ElevatorState.HOME);
            isManualWrist = false;
            //elevatorClaw.setTargetPosition(Constants.cHome);

        }else if(operatorControl.getRawButton(3)){
            elevatorClaw.setElevatorState(ElevatorClaw.ElevatorState.LEVEL1);
            isManualWrist = false;

        }else if(operatorControl.getRawButton(4)){
            elevatorClaw.setElevatorState(ElevatorClaw.ElevatorState.LEVEL2);
            isManualWrist = false;
            
        }else if(operatorControl.getRawButton(2)){
            elevatorClaw.setElevatorState(ElevatorClaw.ElevatorState.LEVEL3);
            isManualWrist = false;
        
        }else if(operatorControl.getRawButton(10)){
            elevatorClaw.setElevatorState(ElevatorClaw.ElevatorState.FEEDER);
            isManualWrist = false;
        }
        //Elevator Adjust
        if(operatorControl.getRawAxis(1) > 0.2){
            elevatorClaw.adjustElevatorPosition(-500);
        }else if(operatorControl.getRawAxis(1) < -0.2){
            elevatorClaw.adjustElevatorPosition(500);
        }
          // Back button is close claw, start button is open claw
        if (operatorControl.getRawButton(7) || driverControl.getRawButton(6)) {
            elevatorClaw.setGripperState(Value.kForward);
            elevatorClaw.disableGripperAuto(true);
        }
        if (operatorControl.getRawButton(8)) {
            elevatorClaw.setGripperState(Value.kReverse);
            elevatorClaw.disableGripperAuto(true);
        }
        // X button brings wrist down, Y button bings wrist up
        if (operatorControl.getPOV() == 270) {
            elevatorClaw.setWristState(Value.kForward);
            elevatorClaw.disableWristAuto(true);
            isManualWrist = true;
        }
        else if(operatorControl.getPOV() == 90) {
            elevatorClaw.setWristState(Value.kReverse);
            elevatorClaw.disableWristAuto(true);
            isManualWrist = true;
        }else if(isManualWrist){
            elevatorClaw.setWristState(Value.kOff);
        }

        // A button is stopper down, B button is stopper up
        if (operatorControl.getPOV() == 0) {
            elevatorClaw.setStopperState(Value.kForward);
            elevatorClaw.disableStopperAuto(true);
        }
        if (operatorControl.getPOV() == 180) {
            elevatorClaw.setStopperState(Value.kReverse);
            elevatorClaw.disableStopperAuto(true);
        }

        if (operatorControl.getRawButton(5)) {
            intake.setIntakeState(Value.kForward);
        }
        if (operatorControl.getRawButton(6)) {
            intake.setIntakeState(Value.kReverse);
        }

        if (operatorControl.getRawAxis(2)>0.1 || driverControl.getRawAxis(2)>0.1){
            intake.setIntakeAll(Intake.IntakeDirection.FORWARD, operatorControl.getRawAxis(2));
            
        }else if(operatorControl.getRawAxis(3)>0.1){
            intake.setIntakeAll(Intake.IntakeDirection.REVERSE, operatorControl.getRawAxis(3));
        }
        else {
            intake.setIntakeAll(Intake.IntakeDirection.OFF, 0);
        }   
        if(driverControl.getRawButton(5)){
            elevatorClaw.pushGamePiece();
        }else{
            elevatorClaw.retractPusher();
        }   
    }
    

   /*  public void manualControls(){
        //Conveyor Manual Control "triggers"
        if(manualControl.getRawAxis(2)>0.1){
            intake.setConveyorSpeed(1);
        }else if(manualControl.getRawAxis(3) > 0.1){
            intake.setConveyorSpeed(-1);
        }
        
        //Flappy Manual Control "Bumpers"
        if(manualControl.getRawButton(5)){
            intake.setFlappySpeed(1);
            //Intake.intakeIn= true;
        }else if(manualControl.getRawButton(6)){
            intake.setFlappySpeed(-1);
        }
        
        //Stopper Manual Control "DPad up & down"
        else if (manualControl.getPOV() == 0){
            elevatorClaw.setStopperState(Value.kForward);
            elevatorClaw.disableStopperAuto(true);

        }else if(manualControl.getPOV() == 180){
            elevatorClaw.setStopperState(Value.kReverse);
            elevatorClaw.disableStopperAuto(true);
        }

        //Wrist Manual Control "DPad left and right"
        else if(manualControl.getPOV() == 270) {
            elevatorClaw.setWristState(Value.kForward);
            elevatorClaw.disableWristAuto(true);
        } else if (manualControl.getPOV() == 90) {
            elevatorClaw.setWristState(Value.kReverse);
            elevatorClaw.disableWristAuto(true);
        }

        //Gripper Manual State "Start and Back"
        else if (manualControl.getRawButton(7)) {
            elevatorClaw.setGripperState(Value.kForward);
            elevatorClaw.disableGripperAuto(true);
        } else if (manualControl.getRawButton(8)) {
            elevatorClaw.setGripperState(Value.kReverse);
            elevatorClaw.disableGripperAuto(true);
        }

        //Elevator Adjusting Manual Control "Left Joystick"
        else if(manualControl.getRawAxis(1) > 0.2){
            elevatorClaw.adjustElevatorPosition(-500);
        }else if(manualControl.getRawAxis(1) < -0.2){
            elevatorClaw.adjustElevatorPosition(500);
        }

        //Intake In and Out Manual Control "Buttons A and"
        else if(manualControl.getRawButton(1)){
            intake.setIntakeState(Value.kForward);
        } else if(manualControl.getRawButton(2)){
            intake.setIntakeState(Value.kReverse);
        }


    }*/
        



    public void testOI() {

        // BL

       /*  if (testJoy1.getRawButton(1)) {
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
        } */

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
        if (testJoy2.getRawButton(7)) {
            elevatorClaw.setGripperState(Value.kForward);
        }
        if (testJoy2.getRawButton(8)) {
            elevatorClaw.setGripperState(Value.kReverse);
        }

        // X button brings wrist down, Y button bings wrist up
        if (testJoy2.getRawButton(3)) {
            elevatorClaw.setWristState(Value.kForward);
        }
        if (testJoy2.getRawButton(4)) {
            elevatorClaw.setWristState(Value.kReverse);
        }

        // A button is stopper down, B button is stopper up
        if (testJoy2.getRawButton(1)) {
            elevatorClaw.setStopperState(Value.kForward);
        }
        if (testJoy2.getRawButton(2)) {
            elevatorClaw.setStopperState(Value.kReverse);
        }

        if (testJoy2.getRawAxis(2)>0.1) {
            intake.setIntakeState(Value.kForward);
        }
        if (testJoy2.getRawAxis(3)>0.1) {
            intake.setIntakeState(Value.kReverse);
        }

       /*  if (testJoy2.getRawButton(5)){
            intake.setIntakeAll(Intake.IntakeDirection.FORWARD);
        }else if(testJoy2.getRawButton(6)){
            intake.setIntakeAll(Intake.IntakeDirection.REVERSE);
        }
        else {
            intake.setIntakeAll(Intake.IntakeDirection.OFF);
        }*/
/* 
        if (testJoy1.getRawButton(1)){
            elevatorClaw.setElevatorPosition(Constants.cHome);
        }

        if (testJoy1.getRawButton(2)){
            elevatorClaw.setElevatorPosition(Constants.cLevel3);
            
        }
*/
       if (testJoy1.getRawButton(1)){
            elevatorClaw.setElevatorPosition(Constants.cHome);
        }else if (testJoy1.getRawButton(2)){
            elevatorClaw.setElevatorPosition(Constants.cLevel1);
        }else if(testJoy1.getRawButton(3)){
            elevatorClaw.setElevatorPosition(Constants.cLevel2);
        }else if(testJoy1.getRawButton(4)){
            elevatorClaw.setElevatorPosition(Constants.cLevel3);
        }else{
            elevatorClaw.calcElevatorSpeed(0);
        }
    }

}
