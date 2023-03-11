package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorClaw;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class armPos extends CommandBase {

    boolean isFinished;

    ElevatorClaw elevatorClaw;

    double position;

    public armPos(double position) {
        elevatorClaw = ElevatorClaw.get_Instance();
        this.position = position;
    }

    @Override
    public void initialize() {
        elevatorClaw.setTargetPosition(position);
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
        return isFinished;
    }

    @Override
    public void execute() {
        isFinished = elevatorClaw.isElevatorAtPosition(1000);
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {

    }
}