package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.*;
import frc.robot.Util.swerveOdometry;

public class Lime {
    private static Lime lime;
    private Drive drive;

    public Lime() {
    }

    public static Lime get_Instance() {
        if (lime == null) {
            lime = new Lime();
        }
        return lime;
    }

    public double getHorOffset() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public void setPipeline(int setPipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(setPipeline);
    }

    public int getPipeline() {
        return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
    }

    /*
     * public double calcDistance(){
     * return (Constants.h2-Constants.h1)/Math.tan((Constants.a1 -
     * NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").
     * getDouble(0))*Math.PI/180)-31;
     * }
     */
    public boolean isSeeTar() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
    }

    public boolean isSeeApril() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0) > 0.3;
    }

    public double getX() {

        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
                .getDoubleArray(new double[6])[0];
    }

    public double getY() {

        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
                .getDoubleArray(new double[6])[1];
    }

    public double getTheta() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
                .getDoubleArray(new double[6])[5];
    }
}
