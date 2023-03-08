package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class Intake {
    private CANSparkMax mIntake;
    private CANSparkMax mFlapperWheels;
    private CANSparkMax mConveyor;

    private DoubleSolenoid sIntake;
    private DigitalInput gamePiecePhotoEye;

    private boolean intakeIn;
    private boolean conveyorOn;

    private Timer conveyorTime;

    public enum IntakeDirection{
        FORWARD,
        OFF,
        REVERSE
    }

    private static Intake intake;

    public static Intake get_Instance(){
        if(intake == null){
            intake = new Intake();
        }
        return intake;
    }
    
    Intake(){
        //Initalize Components
        mIntake = new CANSparkMax(Constants.mIntakeWheels, MotorType.kBrushless);
        mFlapperWheels = new CANSparkMax(Constants.mFlapperWheels, MotorType.kBrushless);
        mConveyor = new CANSparkMax(Constants.mConveyor, MotorType.kBrushless);
        sIntake = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sIntakeID[0], Constants.sIntakeID[1]);
        gamePiecePhotoEye = new DigitalInput(Constants.gamePiecePhotoeye);

        //Initalize State Variables
        intakeIn = true;
        conveyorOn = false;
        conveyorTime = new Timer();
    }

    /**
     * Sets the speed of the intake
     * @param speed the speed of the intake
     */
    public void setIntakeSpeed(double speed){
        mIntake.set(speed);
    }

    /**
     * Sets the state of the intake
     * @param val the position the intake should be in
     */
    public void setIntakeState(Value val){  
        sIntake.set(val);

        //Is the intake in? 
        if(val==Value.kForward) intakeIn = true;
        else intakeIn = false;
    }

    /**
     * Sets all intake necessary components
     * @param dir the direction the intake system should run
     */
    public void setIntakeIntegrated(IntakeDirection dir){
        //Determine intake direction
        if(dir == IntakeDirection.FORWARD){
            setIntakeSpeed(1);
            setFlappySpeed(1);
            setConveyorSpeed(1);

            conveyorTime.restart();
            conveyorOn = true;
        }else if(dir == IntakeDirection.REVERSE){
            setIntakeSpeed(-1);
            setFlappySpeed(-1);
            setConveyorSpeed(-1);
            conveyorOn = false;
        }else{
            setIntakeSpeed(0);
            setFlappySpeed(0);
        }

        //Check if intake is in
        //if so, turn off intake and flappers
        if(intakeIn){
            setIntakeSpeed(0);
            setFlappySpeed(0);
        }
    }

    /**
     * Check if conveyor should keep running 
     * after intaking a game piece
     */
    public void checkConveyorState(){
        if(conveyorOn && (getGamePiecePhotoeye() || conveyorTime.get() > .5)){
            setConveyorSpeed(0);
            conveyorTime.stop();
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
        if(speed == 0) conveyorOn = false;
    }

    public Value getIntakePos(){
        return sIntake.get();
    }
   
    public boolean getGamePiecePhotoeye(){
        return !gamePiecePhotoEye.get();
    }

    public void periodic(){
        checkConveyorState();
    }
}
