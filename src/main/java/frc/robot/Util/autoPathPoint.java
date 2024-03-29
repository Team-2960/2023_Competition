package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class autoPathPoint {
    public final Pose2d pose;
    public final double posTolerance;
    public final boolean isSeeApril;
    public autoPathPoint(Pose2d pose){
        this.pose = pose;
        posTolerance = 0.2;
        isSeeApril = false;
    }
    public autoPathPoint(Pose2d pose, double coordTol, boolean isSeeApril){
        this.pose = pose;
        posTolerance = coordTol;
        this.isSeeApril = isSeeApril;
    }
    public double getX(){
        return pose.getX();
    }
    public double getY(){
        return pose.getY();
    }
    public double getTheta(){
        return pose.getRotation().getDegrees();
    }
    public double getTolerance(){
        return posTolerance;
    }
    public boolean isSeeApril(){
        return isSeeApril;
    }
}
