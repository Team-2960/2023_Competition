package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorClaw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lime;
import frc.robot.subsystems.Intake.IntakeDirection;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class intakeOn extends CommandBase {

    boolean isFinished;

    Intake intake;

    Timer timer;
    double time;

    public intakeOn(double time) {
        intake = Intake.get_Instance();
        timer = new Timer();
        this.time = time;
    }

    @Override
    public void initialize() {
        timer.start();
        intake.setIntakeState(Value.kForward);

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
        return timer.get()> time;
    }

    @Override
    public void execute() {
        intake.setIntakeAll(IntakeDirection.FORWARD, 1);
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        intake.setIntakeAll(IntakeDirection.OFF, 0);
        intake.setIntakeState(Value.kReverse);
        intake.setConveyorSpeed(0);
    }
}