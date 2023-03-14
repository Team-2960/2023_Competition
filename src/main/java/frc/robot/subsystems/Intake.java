package frc.robot.subsystems;

import com.fasterxml.jackson.databind.cfg.ConstructorDetector.SingleArgConstructor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake {
    private CANSparkMax mIntake;
    private CANSparkMax mFlapperWheels;
    private CANSparkMax mConveyor;
    private DoubleSolenoid sIntake;
    private DigitalInput gamePiecePhotoEye;
    private boolean intakeIn;
    private boolean conveyorOn;
    private Timer conveyorTimer;
    private Timer intakeTimer;
    //public boolean intakeIn2;

    private static Intake intake;

    public enum IntakeDirection{
        FORWARD,
        OFF,
        REVERSE
    }

    public static Intake get_Instance(){
        if(intake == null){
            intake = new Intake();
        }
        return intake;
    }
    
    private Intake(){
        mIntake = new CANSparkMax(Constants.mIntakeWheels, MotorType.kBrushless);
        mFlapperWheels = new CANSparkMax(Constants.mFlapperWheels, MotorType.kBrushless);
        mConveyor = new CANSparkMax(Constants.mConveyor, MotorType.kBrushless);
        sIntake = new DoubleSolenoid(18,PneumaticsModuleType.REVPH, Constants.sIntakeID[0], Constants.sIntakeID[1]);
        gamePiecePhotoEye = new DigitalInput(Constants.gamePiecePhotoeye);
        intakeIn = true;
       // intakeIn2 = true;
        conveyorOn = false;
        intakeTimer = new Timer();
        conveyorTimer = new Timer();
        //TODO: check value
        sIntake.set(Value.kReverse);
    }

    /**
     * Sets the speed of the intake
     * @param speed the speed of the intake
     */
    public void setIntakeSpeed(double speed){
        mIntake.set(speed/2);
    }

    /**
     * Sets the state of the intake
     * @param val the position the intake should be in
     */
    public void setIntakeState(Value val){  
        sIntake.set(val);
        if(val==Value.kReverse){
            intakeIn = true;
            //intakeIn2 = true;

        }else{
            intakeIn = false;
            //intakeIn2 = false;
        }

    }
    public void checkIntakeTimer(){
        if(intakeTimer.get()>.25){
            sIntake.set(Value.kOff);
            intakeTimer.reset();
            intakeTimer.stop();
        }
    }

    public void setIntakeAll(IntakeDirection dir){
        if(dir == IntakeDirection.FORWARD){
            setIntakeSpeed(-1);
            setFlappySpeed(1);
            setConveyorSpeed(1);
            conveyorOn = true;
            conveyorTimer.start();
            conveyorTimer.reset();
        }else if(dir == IntakeDirection.REVERSE){
            setIntakeSpeed(1);
            setFlappySpeed(-1);
            setConveyorSpeed(-1);
            conveyorOn = false;
        }else{
            setIntakeSpeed(0);
            setFlappySpeed(0);
            if(!conveyorOn){
                setConveyorSpeed(0);
            }
        }
         
        if(intakeIn){
            setFlappySpeed(0);
            setIntakeSpeed(0);
        }
    }

    public void checkConveyorState(){
        if(conveyorOn){
           if (getGamePiecePhotoeye()){
            conveyorOn =  false;
           }
           else if(conveyorTimer.get() > 1.5){
            conveyorOn = false;
            conveyorTimer.stop();
           }
        }
    }

    /**
     * Sets the speed of the flappers
     * @param speed the speed of the flappers
     */
    public void setFlappySpeed(double speed){
        mFlapperWheels.set(speed);
    }

    /**
     * Sets the speed of the flappers
     * @param speed the speed of the flappers
     */
    public void setConveyorSpeed(double speed){
        mConveyor.set(speed);
        if(speed == 0){
            conveyorOn = false;
        }
    }
    
    public Value getIntakePos(){
        return sIntake.get();
    }
   
    public boolean getGamePiecePhotoeye(){
        return !gamePiecePhotoEye.get();
    }
    public void periodic(){
        checkConveyorState();
        SmartDashboard.putNumber("conveyEncod", mConveyor.getEncoder().getPosition());
    }
}
