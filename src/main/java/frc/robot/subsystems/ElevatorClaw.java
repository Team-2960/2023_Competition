package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
    private DoubleSolenoid sIntake;

    private ElevatorClaw (){
        mRElevator= new TalonFX(Constants.mRElevator);
        mLElevator= new TalonFX(Constants.mLElevator);
        sGripper= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sGripperID[0], Constants.sGripperID[1]);
        sWrist= new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sWristID[0], Constants.sWristID[1]);
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
     * Open Gripper
     * @param val the state of gripper solenoids
     */
    public void setIntakeState(Value val){
        sIntake.set(val);
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
}