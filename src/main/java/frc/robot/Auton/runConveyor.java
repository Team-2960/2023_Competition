package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorClaw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class runConveyor extends CommandBase {

    boolean isFinished;

    Drive drive;
    Intake intake;

    Timer timer;
    public runConveyor() {
        drive = Drive.get_Instance();
        intake = Intake.get_Instance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
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
        return timer.get() > 0.5;
    }

    @Override
    public void execute() {
        intake.setConveyorSpeed(1);
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        intake.setConveyorSpeed(0);
    }
}