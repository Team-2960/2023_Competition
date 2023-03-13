package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class pulseWheels extends CommandBase {

    boolean isFinished;

    Drive drive;

    Timer timer;

    double velX;
    double velY;

    double time;
    public pulseWheels(double time, double velX, double velY) {
        drive = Drive.get_Instance();
        timer = new Timer();
        this.velY = velY;
        this.velX = velX;
        this.time = time;
    }

    @Override
    public void initialize() {
        drive.velX = velX;
        drive.velY = velY;
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
    public void execute() {}

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        timer.stop();
        drive.velY =0;
    }
}