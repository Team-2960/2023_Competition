package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class wait extends CommandBase {

    boolean isFinished;

    Drive drive;

    Timer timer;

    Timer timer2;

    private double time;
    public wait(double time) {
        drive = Drive.get_Instance();
        timer = new Timer();
        this.time = time;
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
        return timer.get() > time;
    }

    @Override
    public void execute() {
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        timer.stop();
        drive.velY =0;
        drive.velX = 0;
    }
}