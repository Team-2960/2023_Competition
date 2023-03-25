package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorClaw;
import frc.robot.subsystems.Lime;

import javax.swing.plaf.metal.MetalBorders.ScrollPaneBorder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

public class alignAndDriveVisionRight extends CommandBase {

    boolean isFinished;

    Drive drive;
    Lime lime;

    Timer timer;

    Timer visionTimer;
    Timer delayTimer;
    Timer alignmentDelay;

    double error = 100000;
    double angleError = 10000;

    double xCoord = 0;
    double baseSpeed = 0;
    double slowSpeed = 0;
    double adjSpeed = 0;
    double slowDownDistance = 0.5;
    double tolerance = 0;
    double dx;
    double tarAngle;
    double limeTol;
    double target = 10;

    public alignAndDriveVisionRight(double xCoord, double baseSpeed, double slowSpeed, double tolerance,
            double tarAngle, double limeTol) {
        drive = Drive.get_Instance();
        lime = Lime.get_Instance();
        timer = new Timer();
        visionTimer = new Timer();
        delayTimer = new Timer();
        alignmentDelay = new Timer();

        this.xCoord = xCoord;

        this.baseSpeed = baseSpeed;
        this.slowSpeed = slowSpeed;
        if (!Drive.isBlueAlliance()) {
            this.xCoord *= -1;
        }
        this.tolerance = tolerance;
        this.tarAngle = tarAngle;
        if (!Drive.isBlueAlliance()) {
            this.tarAngle += 180;
        }
        this.limeTol = limeTol;
    }

    @Override
    public void initialize() {
        if (!Drive.isBlueAlliance()) {
            lime.setPipeline(1);
        } else {
            lime.setPipeline(3);
        }
        timer.reset();
        timer.start();
        delayTimer.reset();
        delayTimer.start();
        visionTimer.reset();
        visionTimer.start();
        alignmentDelay.reset();
        alignmentDelay.start();
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
        return (visionTimer.get() > 0.2 && Math.toDegrees(Math.abs(angleError)) < 10) && Math.abs(dx) < tolerance;
    }

    @Override
    public void execute() {
        if (delayTimer.get() > 0.5) {
            SmartDashboard.putNumber("angle error", Math.toDegrees(angleError));
            SmartDashboard.putNumber("vision timer", visionTimer.get());
            SmartDashboard.putNumber("dx a", dx);
            SmartDashboard.putNumber("signerr", error);
            double currXRobot = drive.getRobotPos().getX();
            double currTheta = Math.toRadians(Drive.getFieldAngle());

            SmartDashboard.putNumber("curr Theta", currTheta);

            double currPosTheta = Math.toRadians(0);
            double currPosPosTheta = Math.toRadians(-360);
            double currPosNegTheta = Math.toRadians(360);

            if (!Drive.isBlueAlliance()) {
                currPosTheta = Math.toRadians(180);
                currPosPosTheta = Math.toRadians(-180);
                currPosNegTheta = Math.toRadians(540);
            }

            double dThetaRegErr = currPosTheta - currTheta;
            double dThetaNegErr = currPosNegTheta - currTheta;
            double dThetaPosErr = currPosPosTheta - currTheta;
            double tarTheta = 0;
            if (Math.abs(dThetaRegErr) > Math.abs(dThetaPosErr) && Math.abs(dThetaNegErr) > Math.abs(dThetaPosErr)) {
                tarTheta = currPosPosTheta;
            } else if (Math.abs(dThetaRegErr) > Math.abs(dThetaNegErr)
                    && Math.abs(dThetaPosErr) > Math.abs(dThetaNegErr)) {
                tarTheta = currPosNegTheta;
            } else {
                tarTheta = currPosTheta;
            }

            double dTheta = tarTheta - currTheta;
            angleError = dTheta;
            double tarPointX = xCoord;
            dx = tarPointX - currXRobot;
            double mag = Math.sqrt(Math.pow(dx, 2));
            double udx = dx / mag;

            if (mag < slowDownDistance) {
                double slope = (baseSpeed - slowSpeed) / slowDownDistance;
                adjSpeed = slowSpeed + mag * slope;
            } else {
                adjSpeed = baseSpeed;
            }

            double velX = adjSpeed * udx;
            drive.velY = velX;
            double sigError = target - lime.getHorOffset();
            if (Math.abs(sigError) < limeTol) {
                visionTimer.start();
            } else {
                visionTimer.reset();
            }
            if (Math.abs(tarAngle - drive.getFieldAngle()) < 5) {
                if (timer.get() > 0.2) {
                    error = Math.abs(target - lime.getHorOffset());
                    // error = Math.max(10, Math.min(1, error));
                    double dir = 1;
                    if (sigError > 0)
                        dir = -1;
                    //I think this is the problem
                    if (!Drive.isBlueAlliance()) {
                        dir *= -1;
                    }
                    drive.velX = dir * (error / 15 + 1 / 15);

                    drive.omega = 0;
                }
            } else {

                double omega = 0;

                // NOTE tarTheta IS IN DEGREES BUT OUTPUT WILL BE IN RAD/SEC
                // This part sets the omega based on how far we are from the desired theta
                if (Math.abs(dTheta) < Constants.thresholdT1) {
                    omega = Constants.tVel1;
                } else if (Math.abs(dTheta) < Constants.thresholdT2) {
                    omega = Constants.tVel2;
                } else if (Math.abs(dTheta) < Constants.thresholdT3) {
                    omega = Constants.tVel3;
                } else {
                    omega = Constants.tVelOutside;
                }

                // THE SIGNS MIGHT NEED TO BE FLIPPED
                // THIS PART OF THE CODE ADJUSTS THE DIRECTION OF THE OMEGA SO THAT IT GOES THE
                // CORRECT DIRECTION BASED ON WHETHER OUR ERROR IS POSITIVE OR NEGATIVE
                if (dTheta < 0) {
                    omega = -1 * omega;
                }

                drive.omega = omega;
            }
        }
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        System.out.println("Finished Command");
        // lime.setPipeline(0);
        drive.velY = 0;
        drive.velX = 0;
        timer.stop();
    }
}