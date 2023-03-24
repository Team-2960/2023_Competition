package frc.robot.Auton;

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

public class maker extends SequentialCommandGroup {
    public maker(String url) throws IOException{
        addCommands(
            new ParallelRaceGroup(
                new pastXPosition(6.2, true),
                new driveInDir(-1, 0)),
            new alignAndDriveVisionRight(-6.55, 1, 0.3, 0.1, 0, 4)
            /*new setVisionPipeline(0),
            new grabGamePiece(),
            new armPos(ElevatorState.LEVEL1),
            new releaseGamePiece(),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new pastXPosition(5.5, false),
                    new armPos(ElevatorState.HOME),
                    new releaseGamePiece(),
                    new intakeOn(6),
                    new setVisionPipeline(2),
                    new armPos(ElevatorState.LEVEL1)),
                new toArrayMaker(2,0.75,1,0.5,0.3,20, Filesystem.getDeployDirectory() + "/3GamePiece.json")),
            new setVisionPipeline(0),
            new grabGamePiece(),
            new armPos(ElevatorState.LEVEL1),
            new releaseGamePiece(),
            new wait(1),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new pastXPosition(5.5, false),
                    new armPos(ElevatorState.HOME),
                    new intakeOn(6),
                    new setVisionPipeline(2),
                    new armPos(ElevatorState.LEVEL1)),
                new toArrayMaker(1.5,0.75,1,0.5,0.3,20, Filesystem.getDeployDirectory() + "/3GamePiece pt2.json")),
            new releaseGamePiece()   */                 
        );
    }
}