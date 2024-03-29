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



public class intakeOff extends CommandBase {

    boolean isFinished;

    Intake intake;

    public intakeOff() {
        intake = Intake.get_Instance();
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
        intake.setIntakeAll(IntakeDirection.OFF, 0);
        return true;
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