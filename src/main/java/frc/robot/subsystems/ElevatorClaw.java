package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorClaw {

    //Instance
    private static ElevatorClaw elevatorClaw;

    //MOTORS
    private TalonFX mRElevator;
    private TalonFX mLElevator;
    //SOLENOIDS
    private DoubleSolenoid sGripper;
    private DoubleSolenoid sWrist;
    private DoubleSolenoid sStopper;
    //Photoeyes
    private DigitalInput lowerPhotoEye;
    private DigitalInput upperPhotoEye;
    //Timers
    private Timer gripperTimer;
    private Timer wristTimer;
    private Timer stopperTimer;

    private boolean enablePID = false;
    private double target = 0;

    private PIDController pElevatorPID;
    private PIDController elvSpeedPID;

    private double elevatorTarget = 0;

    public enum ElevatorState{
        HOME,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        MOVING
    }
    private ElevatorState currentState;
    private ElevatorState targetState;

    private ElevatorClaw (){
        mRElevator= new TalonFX(Constants.mRElevator, "Default Name");
        mLElevator= new TalonFX(Constants.mLElevator, "Default Name");
        //mLElevator.setInverted(true);
        pElevatorPID = new PIDController(Constants.kElevatorp,Constants.kElevatori,Constants.kElevatord);
        elvSpeedPID = new PIDController(Constants.elvSpeedP, Constants.elvSpeedI, Constants.elvSpeedD);

        sGripper= new DoubleSolenoid(18,PneumaticsModuleType.REVPH, Constants.sGripperID[0], Constants.sGripperID[1]);
        sWrist= new DoubleSolenoid(18,PneumaticsModuleType.REVPH, Constants.sWristID[0], Constants.sWristID[1]);
        sStopper= new DoubleSolenoid(18,PneumaticsModuleType.REVPH, Constants.sStopperID[0], Constants.sStopperID[1]);
        lowerPhotoEye = new DigitalInput(Constants.LowerElevatorPhotoeye);
        upperPhotoEye = new DigitalInput(Constants.UpperElevatorPhotoeye);
        currentState = ElevatorState.HOME;
        targetState = ElevatorState.HOME;
        mLElevator.setSelectedSensorPosition(0);
    }

    public static ElevatorClaw get_Instance(){
        if(elevatorClaw == null){
            elevatorClaw = new ElevatorClaw();
        }
        return elevatorClaw;
    }

    /**
     * Open Gripper
     * @param val the state of gripper solenoids
     */
    public void setGripperState(Value val){
        sGripper.set(val);
    }

    /**
     * Open Gripper
     * @param val the state of gripper solenoids
     */
    public void setWristState(Value val){
        sWrist.set(val);
    }

   
    /**
     * sets the speed to both sides of the elevator
     * @param speedR Left speed
     * @param speedL Right speed
     */
    public void setElevator(double speed){
        mLElevator.set(ControlMode.PercentOutput, -speed);
        mRElevator.set(ControlMode.PercentOutput, speed);
    }
    //Elevator move conditions-
    /*public void setElevatorSpeed(double speed){
        if(lowerPhotoEye.get() && speed < 0){
            speed = 0;
        }else if(upperPhotoEye.get() && speed > 0){
            speed = 0;
        }
        mLElevator.set(ControlMode.PercentOutput, speed);
        mRElevator.set(ControlMode.PercentOutput, speed);
   0 }*/
     public void calcElevatorSpeed(double speed){

        double baseSpeed = feedForwardElevator(speed);

        double encoderValue =  (mLElevator.getSelectedSensorVelocity() - mRElevator.getSelectedSensorVelocity())/2;
        double calcSpeed = baseSpeed + elvSpeedPID.calculate(encoderValue, speed);
        SmartDashboard.putNumber("calculatedSpeed", calcSpeed);
        setElevator(-calcSpeed);
    }

    public double feedForwardElevator(double tarVelocity){
        return tarVelocity * 0.0000573171 + 0.06;
    }
    /* 
    public void setElevatorState(ElevatorState position){
        double targetPosition = 0;
        targetState = position;
        if(position == ElevatorState.LEVEL1){
            targetPosition = Constants.cLevel1;
        }
        else if(position == ElevatorState.LEVEL2){
            targetPosition = Constants.cLevel2;
        }
        else if(position == ElevatorState.LEVEL3){
            targetPosition = Constants.cLevel3;
        }
        else if(position == ElevatorState.HOME){
            targetPosition = Constants.cHome;
        }
        if(currentState != targetState){
            currentState = ElevatorState.MOVING;
            setElevatorPosition(targetPosition);
        }
    }*/
    public void setTargetPosition(double target){
        elevatorTarget = target;
    }
    public void setElevatorPosition(double position){
        double currentPos = (mLElevator.getSelectedSensorPosition() - mRElevator.getSelectedSensorPosition())/2;
        double diff = currentPos - position;
        double far = 10000;
        double tolerance = 500;
        double direction = 1;
        if(diff > 0) direction = -1;
        if(elevatorTarget < 1000 && currentPos < 1000){
            setElevator(0);
        }
        else if(Math.abs(diff) <= tolerance){
            calcElevatorSpeed(0);
            currentState = targetState;
        }
        else if(Math.abs(diff) >= far){
            calcElevatorSpeed(5000 * direction);
        }
        else if(Math.abs(diff) < far){
            calcElevatorSpeed(500 * direction);
        }
    } 
    public void resetElevator(){
        mRElevator.setSelectedSensorPosition(0);
        mLElevator.setSelectedSensorPosition(0);
    }


    
    public void calcPID(double tar){
        double currentPos = mLElevator.getSelectedSensorPosition();
        double speed;
        speed = pElevatorPID.calculate(currentPos,tar);
        SmartDashboard.putNumber("ElevatorPIDspeed", speed);
        SmartDashboard.putNumber("ElevatorCurrentPos", currentPos);
        SmartDashboard.putNumber("ElevatorTargetPos", tar);
        setElevator(-speed);
    }
    /* 
    public void setElevatorPosition(double targetPosition){
            target = targetPosition;
            enablePID = true;
    }
*/

    //Elevator move conditions (stopper)
    public void checkStopperPosition(){
        if(targetState == ElevatorState.HOME && currentState == ElevatorState.HOME){
            setStopperState(Value.kForward);
        }else{
            setStopperState(Value.kReverse);
        }
    }
    public void checkGripperPosition(){
        if(currentState == ElevatorState.HOME && targetState != ElevatorState.HOME){
            setGripperState(Value.kForward);
        }else if(targetState == ElevatorState.HOME && currentState != ElevatorState.HOME){
            setGripperState(Value.kForward);
        }
    }

    public Value getWristPos(){
        return sWrist.get();
    }

    public Value getGripperPos(){
        return sGripper.get();
    }
    /**
     * Sets the state of the stopper
     * @param val the position the intake should be in
     */
    public void setStopperState(Value val){
        sStopper.set(val);
    }
    public Value getStopperPos(){
        return sStopper.get();
    }
    public boolean getLowerPhotoEye(){
        return lowerPhotoEye.get();
    }
    public boolean getUpperPhotoEye(){
        return upperPhotoEye.get();
    }

    //Timers for Solenoids
    public void setGripperState2(boolean state){
        if(state){
            sGripper.set(Value.kForward);
        }else{
            sGripper.set(Value.kReverse);
        }
        gripperTimer.reset();
        gripperTimer.start();
    }
    public void checkGripperTimer(){
        if(gripperTimer.get()>.25){
            sGripper.set(Value.kOff);
            gripperTimer.reset();
            gripperTimer.stop();
        }
    }

    public void setWristState2(boolean state){
        if(state){
            sWrist.set(Value.kForward);
        }else{
            sWrist.set(Value.kReverse);
        }
        wristTimer.reset();
        wristTimer.start();
    }
    public void checkWristTimer(){
        if(wristTimer.get()>.25){
            sWrist.set(Value.kOff);
            wristTimer.reset();
            wristTimer.stop();
        }
    }

    public void setStopperState2(boolean state){
        if(state){
            sStopper.set(Value.kForward);
        }else{
            sStopper.set(Value.kReverse);
        }
        stopperTimer.reset();
        stopperTimer.start();
    }
    public void checkStopperTimer(){
        if(stopperTimer.get()>.25){
            sStopper.set(Value.kOff);
            stopperTimer.reset();
            stopperTimer.stop();
        }
    }
    public void periodic(){
        setElevatorPosition(elevatorTarget);
        //Right elevator is negative
        SmartDashboard.putNumber("elevatorEncoder1", mLElevator.getSelectedSensorVelocity());
        SmartDashboard.putNumber("elevatorEncoder2", mRElevator.getSelectedSensorVelocity());
        SmartDashboard.putNumber("elevatorPosition1", mLElevator.getSelectedSensorPosition());
        SmartDashboard.putNumber("elevatorPosition2", mRElevator.getSelectedSensorPosition());
        if (enablePID){
        //    calcPID(target);
        }
    if(!lowerPhotoEye.get()){
        resetElevator();
    }
    }
}