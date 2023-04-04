package frc.robot.Auton.Autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class score2Tableside extends SequentialCommandGroup {
    public score2Tableside(String url) throws IOException {
        addCommands(
                new grabGamePiece(),
                new ParallelCommandGroup(
                        new flapperDoorDown(),
                        new wristUp(),
                        new intakeOn(0.25)),
                new armPos(ElevatorState.LEVEL3),
                new wait(0.5),
                new releaseGamePiece(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new wait(1),
                                new armPos(ElevatorState.HOME),
                                new intakeOn(4),
                                new ParallelCommandGroup(

                                        new armPos(ElevatorState.LEVEL3),
                                        new SequentialCommandGroup(
                                                new wait(0.75),
                                                new runConveyor(0.2)
                                        )
                                )
                        ),
                        new toArrayMaker(2.05, 0.75, 1.5, 0.5, 0.3, 20,
                                Filesystem.getDeployDirectory() + "/score2Tableside.json")),
                new wait(0.5),
                new releaseGamePiece());
    }
}