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


public class leftCube extends SequentialCommandGroup{
    public leftCube(String url) throws IOException{
        addCommands(
            new grabGamePiece(),
            new ParallelCommandGroup(
                    new flapperDoorDown(),
                    new wristUp()),
            new armPos(ElevatorState.LEVEL3),
            new releaseGamePiece(),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new pastXPosition(5.5, false),
                    new armPos(ElevatorState.HOME)),
                    new toArrayMaker(1.75,0.75,0.5,0.5,0.3,20, Filesystem.getDeployDirectory() + "/outsideZoneLeft.json"))
        );
    }
}