package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class pastYPosition extends CommandBase {

    boolean isFinished;

    private Drive drive;

    private Timer timer;

    private double y;

    public pastYPosition(double y) {
        drive = Drive.get_Instance();
        this.y = y;
    }

    @Override
    public void initialize() {
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
        return Math.abs(drive.so.getRobotPose().getY()) > y;
    }

    @Override
    public void execute() {
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
    }
}