package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class xWheels extends CommandBase {

    boolean isFinished;

    Drive drive;

    Timer timer;
    public xWheels() {
        drive = Drive.get_Instance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        drive.velX = 0;
        drive.velY = 0;
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
        return timer.get() > 0.75;
    }

    @Override
    public void execute() {
        Drive.isXWheels = true;
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        Drive.isXWheels = false;
    }
}