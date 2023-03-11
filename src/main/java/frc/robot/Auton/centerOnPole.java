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



public class centerOnPole extends CommandBase {

    boolean isFinished;

    Drive drive;
    Lime lime;

    Timer timer;

    double error = 100000;
    double angleError = 10000;

    public centerOnPole() {
        drive = Drive.get_Instance();
        lime = Lime.get_Instance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        lime.setPipeline(1);
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
        return Math.abs(error) < 2 && Math.toDegrees(Math.abs(angleError)) < 4;
    }

    @Override
    public void execute() {
        if(timer.get() > 0.2){
            error = Math.abs(15-lime.getHorOffset());
            double dir = 1;
            if(lime.getHorOffset() < 0) dir = -1;
            drive.velX = dir * error/10;
        }
        double currTheta = Math.toRadians(drive.getFieldAngle());
        SmartDashboard.putNumber("curr Theta", currTheta);

        double currPosTheta =Math.toRadians(0);
        double currPosPosTheta = Math.toRadians(-360);
        double currPosNegTheta = Math.toRadians(360);

        double dThetaRegErr = currPosTheta - currTheta;
        double dThetaNegErr = currPosNegTheta - currTheta;
        double dThetaPosErr = currPosPosTheta - currTheta;
        double tarTheta = 0;
        if(Math.abs(dThetaRegErr) > Math.abs(dThetaPosErr) && Math.abs(dThetaNegErr) > Math.abs(dThetaPosErr)){
            tarTheta = currPosPosTheta;
        }else if(Math.abs(dThetaRegErr) > Math.abs(dThetaNegErr) && Math.abs(dThetaPosErr) > Math.abs(dThetaNegErr)){
            tarTheta = currPosNegTheta;
        }else{
            tarTheta = currPosTheta;
        }
        

        double dTheta = tarTheta - currTheta;
        angleError = dTheta;
        double omega = 0;

        //NOTE tarTheta IS IN DEGREES BUT OUTPUT WILL BE IN RAD/SEC
        //This part sets the omega based on how far we are from the desired theta
        if(Math.abs(dTheta) < Constants.thresholdT1){
            omega = Constants.tVel1;
        }else if(Math.abs(dTheta) <Constants.thresholdT2){
            omega = Constants.tVel2;
        }else if(Math.abs(dTheta) < Constants.thresholdT3){
            omega = Constants.tVel3;
        }else{
            omega = Constants.tVelOutside;
        }

        //THE SIGNS MIGHT NEED TO BE FLIPPED
        //THIS PART OF THE CODE ADJUSTS THE DIRECTION OF THE OMEGA SO THAT IT GOES THE CORRECT DIRECTION BASED ON WHETHER OUR ERROR IS POSITIVE OR NEGATIVE
         if(dTheta < 0){
             omega = -1 * omega;
          }
          drive.omega = omega;
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        lime.setPipeline(0);
        drive.velY = 0;
        drive.velX = 0;
        timer.stop();
    }
}