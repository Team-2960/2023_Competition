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

public class cubeAndBalance extends SequentialCommandGroup {
    public cubeAndBalance(String url) throws IOException {
        addCommands(

                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        new flapperDoorDown(),
                                                        new wristUp()),
                                                new armPos(ElevatorState.LEVEL3)),
                                        new wait(3.5)),
                                new releaseGamePiece(),
                                new wait(0.75),
                                new setVisionPipeline(7),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new pastXPosition(0.5, true),
                                                new armPos(ElevatorState.HOME)),
                                        new toArrayMaker(1.75, 0.75, 0.5, 0.5, 0.3, 20,
                                                Filesystem.getDeployDirectory() + "/newPastAndBack.json")),
                                new autoBalance()),
                        new wait(14.6)),
                new xWheels());

    }
}