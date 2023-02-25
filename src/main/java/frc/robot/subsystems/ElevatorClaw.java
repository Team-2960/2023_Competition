package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;

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
        mRElevator= new TalonFX(Constants.mRElevator, "CANivore");
        mLElevator= new TalonFX(Constants.mLElevator, "CANivore");
        sGripper= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sGripperID[0], Constants.sGripperID[1]);
        sWrist= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sWristID[0], Constants.sWristID[1]);
        sStopper= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sStopperID[0], Constants.sStopperID[1]);
        lowerPhotoEye = new DigitalInput(Constants.LowerElevatorPhotoeye);
        upperPhotoEye = new DigitalInput(Constants.UpperElevatorPhotoeye);
        currentState = ElevatorState.HOME;
        targetState = ElevatorState.HOME;
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
        sGripper.set(val);
    }

   
    /**
     * sets the speed to both sides of the elevator
     * @param speedR Left speed
     * @param speedL Right speed
     */
    public void setElevator(double speedR, double speedL){
        mLElevator.set(ControlMode.PercentOutput, speedL);
        mRElevator.set(ControlMode.PercentOutput, speedR);
    }
    //Elevator move conditions
    public void setElevatorSpeed(double speed){
        if(lowerPhotoEye.get() && speed < 0){
            speed = 0;
        }else if(upperPhotoEye.get() && speed > 0){
            speed = 0;
        }
        mLElevator.set(ControlMode.PercentOutput, speed);
        mRElevator.set(ControlMode.PercentOutput, speed);
    }
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
}