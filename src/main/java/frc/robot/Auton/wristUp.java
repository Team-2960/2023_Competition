package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorClaw;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class wristUp extends CommandBase {

    boolean isFinished;

    ElevatorClaw elevatorClaw;

    Timer timer;
    public wristUp() {
        elevatorClaw = ElevatorClaw.get_Instance();
        timer = new Timer();
        System.out.println("xWheels");
    }

    @Override
    public void initialize() {
        elevatorClaw.disableWristAuto(true);
        timer.start();
    }

    /**
     * Returns true if all the commands in this group have been started and have
     * finished.
     * <p>
     * <p>
     * Teams may override this method, although they should probably reference
     * super.isFinished() if they do.
     * </p>
     *
     * @return whether this {@link CommandGroup} is finished
     */
    @Override
    public boolean isFinished() {
        return elevatorClaw.getWristLoc() > Constants.upWristPos;
    }

    @Override
    public void execute() {
        elevatorClaw.setWristState(Value.kForward);
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        elevatorClaw.disableWristAuto(false);
        Drive.isXWheels = false;
    }
}