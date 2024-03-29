// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import javax.print.attribute.standard.Compression;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auton.maker;
import frc.robot.Auton.Autons.balance;
import frc.robot.Auton.Autons.coneAndBalance;
import frc.robot.Auton.Autons.cubeAndBalance;
import frc.robot.Auton.Autons.cubeBalanceAndGrab;
import frc.robot.Auton.Autons.leftCube;
import frc.robot.Auton.Autons.newScore2;
import frc.robot.Auton.Autons.rightCube;
import frc.robot.Auton.Autons.score2;
import frc.robot.Auton.Autons.score2Tableside;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private OI oi;
  private PowerDistribution pdp;
  private Compressor ph;
  private Intake intake;
  private ElevatorClaw elevatorClaw;
  private Drive drive;
  private Command autonCommand;
  private Lime lime;

  public Robot() {
    addPeriodic(() -> {
      drive.updateOdometry();
    }, 0.005);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture(0);
    PortForwarder.add(5801, "limelight.local", 5801);
    oi = OI.get_Instance();
    pdp = new PowerDistribution(Constants.PDH, PowerDistribution.ModuleType.kRev);
    pdp.setSwitchableChannel(true);
    ph = new Compressor(18, PneumaticsModuleType.REVPH);
    intake = Intake.get_Instance();
    elevatorClaw = ElevatorClaw.get_Instance();
    drive = Drive.get_Instance();
    lime = Lime.get_Instance();
    // ph.disable();
    drive.coastMode();
    try {
      autonCommand = new newScore2("lol this does nothing");
    } catch (IOException e) {

      e.printStackTrace();
    }
  }

  @Override
  public void robotPeriodic() {
    drive.updateOdometry();
    CommandScheduler.getInstance().run();

    elevatorClaw.periodic();
  }

  @Override
  public void autonomousInit() {
    elevatorClaw.setStopperDelay(0.75);
    drive.breakMode();
    elevatorClaw.setElevatorCoastMode();
    if (autonCommand != null)
      autonCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    drive.autonUpdate();
  }

  @Override
  public void teleopInit() {
    elevatorClaw.setStopperDelay(0.75);
    lime.setPipeline(0);
    elevatorClaw.setElevatorCoastMode();
  }

  @Override
  public void teleopPeriodic() {
    oi.regularOI();
    drive.periodicTele();
    // oi.testOI();
    // oi.soberOI();
    intake.periodic();
    elevatorClaw.periodic();
  }

  @Override
  public void disabledInit() {
    drive.coastMode();
    elevatorClaw.setElevatorBrakeMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    // elevatorClaw.periodic();
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
