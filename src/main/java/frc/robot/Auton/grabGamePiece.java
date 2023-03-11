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



public class grabGamePiece extends CommandBase {

    boolean isFinished;

    ElevatorClaw elevatorClaw;

    Timer timer;

    public grabGamePiece() {
        elevatorClaw = ElevatorClaw.get_Instance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
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
        return timer.get() > 0.1;
    }

    @Override
    public void execute() {
        elevatorClaw.setGripperState(Value.kReverse);
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        timer.stop();
    }
}