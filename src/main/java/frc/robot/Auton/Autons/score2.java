package frc.robot.Auton.Autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Auton.*;
import frc.robot.subsystems.ElevatorClaw.ElevatorState;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;

public class score2 extends SequentialCommandGroup {
    public score2(String url) throws IOException {
        addCommands(
                new grabGamePiece(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new armPos(ElevatorState.LEVEL3),
                                new setVisionPipeline(1)),
                        new SequentialCommandGroup(
                                new wait(1.5),
                                new ParallelRaceGroup(
                                        new pastXPosition(6.3, true),
                                        new driveInDir(-1, 0)),
                                new alignAndDriveVisionRight(-6.55, 1, 0.3, 0.1, 0, 6))),
                new releaseGamePiece(),
                new wait(0.2),
                new setVisionPipeline(0),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new pastXPosition(5.5, false),
                                new SequentialCommandGroup(
                                        new armPos(ElevatorState.HOME),

                                        new intakeOn(2.5)),
                                new armPos(ElevatorState.LEVEL3)),
                        new SequentialCommandGroup(
                                new toArrayMaker(2.05, 0.75, 1.5, 0.5, 0.3, 20,
                                        Filesystem.getDeployDirectory() + "/score2.json"),
                                new driveUntilApril(-2, 0))),
                new ParallelRaceGroup(
                        new toArrayMaker(2.05, 0.75, 1.5, 0.5, 0.3, 20,
                                Filesystem.getDeployDirectory() + "/score2 pt2.json"), // needs to be 10000
                        new wait(2.5)),
                new releaseGamePiece());
    }
}