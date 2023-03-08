package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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
    //PID
    private PIDController pElevatorRateControl;

    public enum ElevatorState{
        HOME,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        MOVING,
        MANUAL,
        DRIVING
    }
    private ElevatorState currentState;
    private ElevatorState targetState;

    private double targetPosition;
    private boolean foundTarget;
    private boolean atTarget;
    private Timer movingTimer;
    private Timer atTargerTimer;

    private boolean autoPositionEnabled;

    private ElevatorClaw (){
        //Initialize Components
        mRElevator= new TalonFX(Constants.mRElevator, "CANivore");
        mLElevator= new TalonFX(Constants.mLElevator, "CANivore");
        sGripper= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sGripperID[0], Constants.sGripperID[1]);
        sWrist= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sWristID[0], Constants.sWristID[1]);
        sStopper= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sStopperID[0], Constants.sStopperID[1]);
        lowerPhotoEye = new DigitalInput(Constants.LowerElevatorPhotoeye);
        upperPhotoEye = new DigitalInput(Constants.UpperElevatorPhotoeye);
        pElevatorRateControl = new PIDController(Constants.kElevRateP,Constants.kElevRateI, Constants.kElevRateD);

        //Setup elevator motors
        mRElevator.follow(mLElevator);
        //mRElevator.setInverted(TalonFXInvertType.OpposeMaster); //Does the right motor need to be inverted?
        mLElevator.configForwardSoftLimitThreshold(Constants.elevatorFwdEncLimit);
        mLElevator.configReverseSoftLimitThreshold(Constants.elevatorRevEncLimit);
        
        //Initalize state variables
        currentState = ElevatorState.HOME;
        targetState = ElevatorState.HOME;

        foundTarget = false;
        atTarget = true;
        autoPositionEnabled = true;

        movingTimer = new Timer();
        atTargerTimer = new Timer();
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
     * sets the speed to the elevator
     * @param speed sets the speed
     */
    public void setElevatorSpeed(double speed){
        if(lowerPhotoEye.get() && speed < 0){
            speed = 0;
        }
        mLElevator.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Calculate motor speed based on desired ticks per second
     * @param targetSpeed desired rate in ticks per second
     */
    public void calcElevatorSpeed(double targetSpeed){
        double currEncoderVel = mLElevator.getSelectedSensorVelocity();
        double speed; 
        speed = pElevatorRateControl.calculate(currEncoderVel, targetSpeed);

        setElevatorSpeed(speed);
    }

    /**
     * Set desired destination of elevator
     * @param state target state of elevator
     */
    public void setElevatorPosition(ElevatorState state){
       targetState = state;
       if(state == ElevatorState.HOME) targetPosition = Constants.home;
       else if (state == ElevatorState.LEVEL1) targetPosition = Constants.level1;
       else if (state == ElevatorState.LEVEL2) targetPosition = Constants.level2;
       else if (state == ElevatorState.LEVEL3) targetPosition = Constants.level3;

       if(targetState != currentState && targetState != ElevatorState.DRIVING && targetState != ElevatorState.MANUAL){
            currentState = ElevatorState.MOVING;
            movingTimer.restart();
            atTarget = false;
            foundTarget = false;
       }

    }

    public void addtoElevatorPosition(double position){
        targetPosition = targetPosition + position;

        atTarget = false;
        foundTarget = false;
        movingTimer.restart();
    }

    private void gotoElevatorPosition(){
        double currentPos = mLElevator.getSelectedSensorPosition();
        double errorPos = Math.abs(targetPosition - currentPos);
        double rate = 0; 
        double rampUp;
        double rampDown;

        int dir = 1;

        if(targetPosition < currentPos) dir = -1;

        if(errorPos < Constants.tolerance){
            rate = 0;
            if(!foundTarget){
                foundTarget = true;
                atTargerTimer.restart();
            }else if(atTargerTimer.get()>.1){
                foundTarget = false;
                atTarget = true;
                currentState = targetState;
            }

        }else{
            foundTarget = false;

            rampUp = movingTimer.get()*Constants.elevRampUp;
            rampDown = errorPos*Constants.elevRampDown;

            rate = Math.min(rampUp, rampDown);
            rate = Math.min(rate, Constants.elevMaxRate);
        }
      
        calcElevatorSpeed(rate*dir);
    }

    /**
     * Determine desired stopper position
     * 
     */
    private void autoStopperPosition(){
        if(targetState == ElevatorState.HOME && currentState == ElevatorState.HOME){
            setStopperState(Value.kForward);
        }else{
            setStopperState(Value.kReverse);
        }
    }

    /**
     * Close  gripper in moving to/from HOME
     * So we don't hit the gear boxes
     */
    private void checkGripperPosition(){
        if(currentState == ElevatorState.HOME && targetState != ElevatorState.HOME){
            setGripperState(Value.kForward);
        }else if(targetState == ElevatorState.HOME && currentState != ElevatorState.HOME){
            setGripperState(Value.kForward);
        }
    }

    private void autoWristPosition(){
        Value currentVal = getWristPos();
        Value targetVal;

        if(targetState == ElevatorState.HOME) targetVal = Value.kReverse;
        else if(targetState == ElevatorState.LEVEL1) targetVal = Value.kReverse;
        else if(targetState == ElevatorState.LEVEL2) targetVal = Value.kForward;
        else if(targetState == ElevatorState.LEVEL3) targetVal = Value.kForward;

        if(currentVal == Value.kReverse && targetVal == Value.kForward){

        }else if

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
        return !lowerPhotoEye.get();
    }
    public boolean getUpperPhotoEye(){
        return upperPhotoEye.get();
    }

    public void periodic(){

        checkGripperPosition();
        autoStopperPosition();

        if(autoPositionEnabled && !atTarget){
            gotoElevatorPosition();
        }

    }

}