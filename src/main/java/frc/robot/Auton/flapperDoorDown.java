package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorClaw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lime;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;



public class flapperDoorDown extends CommandBase {

    boolean isFinished = false;

    ElevatorClaw elevatorClaw;

    Timer timer;
    public flapperDoorDown() {
        elevatorClaw = ElevatorClaw.get_Instance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        elevatorClaw.disableStopperAuto(true);
        elevatorClaw.setStopperState(Value.kReverse);
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
        return isFinished;
    }

    @Override
    public void execute() {
        elevatorClaw.setStopperState(Value.kReverse);
        isFinished = true;
    }

    /**
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
    }
}