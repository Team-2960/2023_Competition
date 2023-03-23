package frc.robot.Util;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lime;

public class swerveOdometry {
    static double x;
    static double y;
    static double theta;
    static swerveOdometry so;
    static Lime lime;

    // output
    DoubleLogEntry xLog;
    DoubleLogEntry yLog;
    DoubleLogEntry tLog;

    // intermediary
    DoubleLogEntry fwdCos;
    DoubleLogEntry fwdSin;
    DoubleLogEntry strSin;
    DoubleLogEntry strCos;

    // input
    DoubleLogEntry BLA;
    DoubleLogEntry BLD;
    DoubleLogEntry BRA;
    DoubleLogEntry BRD;
    DoubleLogEntry FLA;
    DoubleLogEntry FLD;
    DoubleLogEntry FRA;
    DoubleLogEntry FRD;

    private Drive drive;

    private SwerveDriveOdometry wpilibSO;

    private Pose2d robotPose;

    public swerveOdometry(SwerveDriveKinematics m_kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modPos) {
        // LOGGING
        DataLog log = DataLogManager.getLog();
        xLog = new DoubleLogEntry(log, "/output/x");
        yLog = new DoubleLogEntry(log, "/output/y");
        tLog = new DoubleLogEntry(log, "/output/t");

        fwdCos = new DoubleLogEntry(log, "/int/fwdCos");
        strSin = new DoubleLogEntry(log, "/int/strSin");
        strCos = new DoubleLogEntry(log, "/int/strCos");
        fwdSin = new DoubleLogEntry(log, "/int/fwdSin");

        BLA = new DoubleLogEntry(log, "/input/BLA");
        BLD = new DoubleLogEntry(log, "/input/BLD");
        BRA = new DoubleLogEntry(log, "/input/BRA");
        BRD = new DoubleLogEntry(log, "/input/BRD");
        FLA = new DoubleLogEntry(log, "/input/FLA");
        FLD = new DoubleLogEntry(log, "/input/FLD");
        FRA = new DoubleLogEntry(log, "/input/FRA");
        FRD = new DoubleLogEntry(log, "/input/FRD");
        x = 0;
        y = 0;
        theta = 0;
        lime = Lime.get_Instance();
        robotPose = new Pose2d();
        wpilibSO = new SwerveDriveOdometry(m_kinematics, gyroAngle, modPos);
    }

    public static double sumDistances(ArrayList<Pose2d> wayPoints, int startingIndex, double currX, double currY) {
        double totalDistance = 0;
        double x1;
        double y1;
        x1 = currX;
        y1 = currY;

        double x2 = wayPoints.get(startingIndex).getX();
        double y2 = wayPoints.get(startingIndex).getY();

        double distance = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        totalDistance += distance;

        for (int i = startingIndex + 1; i < wayPoints.size() - 1; i++) {
            x1 = wayPoints.get(i).getX();
            y1 = wayPoints.get(i).getY();

            x2 = wayPoints.get(i + 1).getX();
            y2 = wayPoints.get(i + 1).getY();

            distance = Math.sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
            totalDistance += distance;
        }

        return totalDistance;
    }

    public void updateOdometry2(SwerveModulePosition[] modPos, Rotation2d gyroAngle) {

        // Update the pose
        robotPose = wpilibSO.update(gyroAngle, modPos);
    }

    public void updateOdometry1(SwerveModuleState fl, SwerveModuleState fr, SwerveModuleState bl, SwerveModuleState br,
            double gyroHeadingRad, double timeStep) {
        gyroHeadingRad = 360 - Drive.reduceAngle(gyroHeadingRad);
        gyroHeadingRad *= (Math.PI / 180);

        // Y Components of Each Module
        double bfl = fl.speedMetersPerSecond * Math.sin(fl.angle.getRadians());
        double bfr = fr.speedMetersPerSecond * Math.sin(fr.angle.getRadians());

        double arl = bl.speedMetersPerSecond * Math.sin(bl.angle.getRadians());
        double arr = br.speedMetersPerSecond * Math.sin(br.angle.getRadians());

        // X Components of Each Module
        double dfl = fl.speedMetersPerSecond * Math.cos(fl.angle.getRadians());
        double cfr = fr.speedMetersPerSecond * Math.cos(fr.angle.getRadians());

        double drl = bl.speedMetersPerSecond * Math.cos(bl.angle.getRadians());
        double crr = br.speedMetersPerSecond * Math.cos(br.angle.getRadians());

        // Simplification
        double a = (arr + arl) / 2;

        double b = (bfl + bfr) / 2;

        double c = (cfr + crr) / 2;

        double d = (dfl + drl) / 2;

        // The length and width of the robot
        double l = 0.5842;

        double w = 0.5842;

        // Find our rotational components idfk
        double rot1 = (b - a) / l;

        double rot2 = (c - d) / w;

        double rot = (rot1 + rot2) / 2;

        // calc forward and strafe components of the robots movement
        double fwd1 = rot * (l / 2) + a;

        double fwd2 = -rot * (l / 2) + b;

        double fwd = (fwd1 + fwd2) / 2;

        double str1 = rot * (w / 2) + c;

        double str2 = -rot * (w / 2) + d;

        double str = (str1 + str2) / 2;
        SmartDashboard.putNumber("fwd2", fwd * Math.cos(gyroHeadingRad));
        SmartDashboard.putNumber("str2", str * Math.cos(gyroHeadingRad));
        SmartDashboard.putNumber("fwd", str * Math.sin(gyroHeadingRad));
        SmartDashboard.putNumber("str", fwd * Math.sin(gyroHeadingRad));
        double dy = (fwd * Math.cos(gyroHeadingRad) + str * Math.sin(gyroHeadingRad));
        double dx = (str * Math.cos(gyroHeadingRad) - fwd * Math.sin(gyroHeadingRad));// Maybe use + instead of -

        x += dx * timeStep;
        y += dy * timeStep;
        theta = gyroHeadingRad;
        /*
         * xLog.append(x);
         * yLog.append(y);
         * tLog.append(theta);
         * 
         * strCos.append(str * Math.cos(gyroHeadingRad));
         * strSin.append(str * Math.sin(gyroHeadingRad));
         * fwdCos.append(fwd * Math.cos(gyroHeadingRad));
         * fwdSin.append(- fwd * Math.sin(gyroHeadingRad));
         * 
         * BLA.append(bl.angle.getDegrees());
         * BLD.append(bl.speedMetersPerSecond);
         * BRA.append(br.angle.getDegrees());
         * BRD.append(br.speedMetersPerSecond);
         * FLA.append(fl.angle.getDegrees());
         * FLD.append(fl.speedMetersPerSecond);
         * FRA.append(fr.angle.getDegrees());
         * FRD.append(fr.speedMetersPerSecond);
         */

        SmartDashboard.putBoolean("is see april", lime.isSeeApril());
        if (lime.isSeeApril() && lime.getPipeline() != 1) {
            try {
                x = lime.getX();
                y = lime.getY();
            } catch (Exception e) {

            }
            /*
             * theta = Math.toRadians(lime.getTheta());
             * while (theta > 2* Math.PI) {
             * theta = theta - 2* Math.PI;
             * }
             * while (theta < 0) {
             * theta = theta + 2* Math.PI;
             * }
             * Drive.setNavX(Math.toDegrees(theta) - drive.navX.getAngle());
             */
        }
    }

    public Pose2d getRobotPose() {
        return new Pose2d(x, y, new Rotation2d(theta));
    }

    public Pose2d getNewRobotPose() {
        return robotPose;
    }

    public String toString() {
        String str = "x: " + x + "\n" +
                "y: " + y + "\n" +
                "Angle:" + (theta * (180 / Math.PI)) + "\n";
        return str;
    }

    public void resetOdometry() {
        x = 0;
        y = 0;
        theta = 0;
    }

    public void setOdometry(double newX, double newY, double newTheta) {
        x = newX;
        y = newY;
        theta = newTheta;
    }
}
