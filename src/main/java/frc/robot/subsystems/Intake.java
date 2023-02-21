package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class Intake {
    private CANSparkMax mIntake;
    private CANSparkMax mFlapperWheels;
    private CANSparkMax mConveyor;
    private DoubleSolenoid sIntake;
    private DoubleSolenoid sStopper;

    

    private static Intake intake;

    public static Intake get_Instance(){
        if(intake == null){
            intake = new Intake();
        }
        return intake;
    }
    
    Intake(){
        mIntake = new CANSparkMax(Constants.mIntakeWheels, MotorType.kBrushless);
        mFlapperWheels = new CANSparkMax(Constants.mFlapperWheels, MotorType.kBrushless);
        mConveyor = new CANSparkMax(Constants.mConveyor, MotorType.kBrushless);
        sIntake = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sIntakeID[0], Constants.sIntakeID[1]);
        sStopper= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sStopperID[0], Constants.sStopperID[1]);
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
    }

    /**
     * Sets the state of the stopper
     * @param val the position the intake should be in
     */
    public void setStopperState(Value val){
        sStopper.set(val);
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
    }
}
