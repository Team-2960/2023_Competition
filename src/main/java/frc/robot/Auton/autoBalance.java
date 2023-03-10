package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class autoBalance extends CommandBase {

    boolean isFinished;

    Drive drive;

    Timer timer;

    Timer timer2;
    public autoBalance() {
        drive = Drive.get_Instance();
        timer = new Timer();
        timer2 = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
        timer2.start();
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
        double pitch = drive.navX.getRoll();
        if(Math.abs(pitch) > 2){
            timer.reset();
            timer.start();
        }
        if(Math.abs(drive.navX.getRawGyroX()) > 3){
            timer2.start();
            if(pitch > 5 && timer2.get() > 0.1){
                drive.velY = 0.15;
            }else if(pitch < -5 && timer2.get() > 0.1){
                drive.velY = -0.15;
            }
        }else if(pitch > 7){
            timer2.reset();
            timer2.start();
            drive.velY = -0.65;
        }else if(pitch < -7){
            timer2.reset();
            timer2.start();
            drive.velY = 0.65;
        }
        else{
            timer2.reset();
            timer2.start();
            drive.velY = 0;
        }
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        timer.stop();
        drive.velY =0;
    }
}