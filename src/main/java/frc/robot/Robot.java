// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.Compression;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private OI oi;
  private PowerDistribution pdp;
  private Compressor ph;
  private Intake intake;
  private ElevatorClaw elevatorClaw;
  private Drive drive;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //PortForwarder.add(5801, "limelight.local", 5801);
    oi = OI.get_Instance();
    pdp = new PowerDistribution(Constants.PDH, PowerDistribution.ModuleType.kRev);
    pdp.setSwitchableChannel(true);
    ph = new Compressor(18,PneumaticsModuleType.REVPH);
    intake = Intake.get_Instance();
    elevatorClaw = ElevatorClaw.get_Instance();
    drive = Drive.get_Instance();
    ph.disable();
    drive.coastMode();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    oi.regularOI();
    drive.periodicTele();
    //oi.testOI();
    //oi.soberOI();
    intake.periodic();
    elevatorClaw.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    //elevatorClaw.periodic();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
