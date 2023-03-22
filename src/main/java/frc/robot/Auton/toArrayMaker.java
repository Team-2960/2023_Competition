package frc.robot.Auton;

import java.io.*;
import java.util.ArrayList;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.Util.swerveOdometry;
import frc.robot.Util.*;

public class toArrayMaker  extends CommandBase {
    ObjectMapper objectMapper;
    ArrayList<autoPathPoint> listOfCoords;

    ArrayList<Pose2d> wayPoints;
    static int currIndexInner;
    static int currIndexOuter;

    //new method vars
    ArrayList<Boolean> crit;
    double critTolerance = 0.2;
    double nonCritTolerance = 0.5;

    int currTarIndex = 0;

    double slowSpeed;

    double critDisTolerance = 0.2;
    double regDisTolerance = 0.4;



    Drive drive;
    double baseSpeed;
    double slowDownDistance;
    double toleranceOuter;
    double toleranceInner;
    double angTolerance;

    //Suppression Vars
    double prevudxO = 0.5;
    double prevudyO = 0.5;
    double prevudxI = 0.5;
    double prevudyI = 0.5;

    //New Suppression Vars
    double prevFinalX = 1;
    double prevFinalY = 1;
    double uVectorFinalX = 0;
    double uVectorFinalY = 0;

    int numInRange = 1000;
    double prevucdx = 0.5;
    double prevucdy = 0.5;
    boolean isInRange = true;

    boolean isFinish = false;
    public toArrayMaker(double baseSpeed, double slowSpeed, double slowDownDistance, double toleranceOuter, double toleranceInner, double angTolerance, String URL){
        //Create ArrayList from JSON File
        objectMapper = new ObjectMapper();
        JsonNode node = null;
        try {
            node = objectMapper.readValue(new File(URL), JsonNode.class);
        } catch (JsonParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (JsonMappingException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        
        int sizeOfPath = node.get(0).get("size").asInt();

        listOfCoords = new ArrayList<autoPathPoint>();

        for(int i = 1; i <= sizeOfPath; i++){
            double x = node.get(i).get("x").asDouble();
            double y = node.get(i).get("y").asDouble();
            double theta = node.get(i).get("theta").asDouble();
            double tolerance = node.get(i).get("tolerance").asDouble();
            boolean isSeeApril = node.get(i).get("isSeeApril").asBoolean();
            autoPathPoint tempPoint = new autoPathPoint(new Pose2d(x,y,Rotation2d.fromDegrees(theta)), tolerance, isSeeApril);
            listOfCoords.add(tempPoint);
        }



        this.baseSpeed = baseSpeed;
        this.slowSpeed = slowSpeed;
        this.slowDownDistance = slowDownDistance;
        this.toleranceOuter = toleranceOuter;
        this.toleranceInner = toleranceInner;
        this.angTolerance = Rotation2d.fromDegrees(angTolerance).getRadians();
        currIndexInner = 0;
        currIndexOuter = 0;
        drive = Drive.get_Instance();
        System.out.println("To array");

    }

    @Override
    public void initialize() {

        currTarIndex = 0;
    }

    @Override
    public boolean isFinished() {
        return isFinish;
    }
    @Override
    public void execute() {
                
        double currXRobot = drive.getRobotPos().getX();
        double currYRobot = drive.getRobotPos().getY();

        double tarPointX = listOfCoords.get(currTarIndex).getX();
        if(!Drive.isBlueAlliance()){
            tarPointX *= -1;
        }
        double tarPointY = listOfCoords.get(currTarIndex).getY();

        SmartDashboard.putNumber("tarx", tarPointX);
        SmartDashboard.putNumber("tary", tarPointY);


        double dx = tarPointX - currXRobot;
        double dy = tarPointY - currYRobot;
        double mag = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        double udx = dx/mag;
        double udy = dy/mag;

        double adjSpeed = baseSpeed;
        double distLeft = mag;
        for(int i = currTarIndex+1; i < listOfCoords.size()-1; i++){
            double dyt = listOfCoords.get(i).getY() - listOfCoords.get(i+1).getY();
            double dxt = listOfCoords.get(i).getX() - listOfCoords.get(i+1).getX();
            double magt = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            distLeft += magt;
        }

        if(distLeft < slowDownDistance){
            double slope = (baseSpeed - slowSpeed)/slowDownDistance;
            adjSpeed = slowSpeed + distLeft * slope;
        }

        double velY = adjSpeed * udx;
        double velX = adjSpeed * udy;

        //OMEGA CALCS
        //Add Step down function to manage omega make sure to include options for camera tracking
        double currTheta = Math.toRadians(drive.getFieldAngle());
        SmartDashboard.putNumber("curr Theta", currTheta);

        double currPosTheta =Math.toRadians(listOfCoords.get(currTarIndex).getTheta());
        double currPosPosTheta = Math.toRadians(listOfCoords.get(currTarIndex).getTheta() - 360);
        double currPosNegTheta = Math.toRadians(listOfCoords.get(currTarIndex).getTheta() + 360);
        if(!Drive.isBlueAlliance()){
            currPosTheta += Math.toRadians(180);
            currPosPosTheta += Math.toRadians(180);
            currPosNegTheta += Math.toRadians(180);
        }

        double dThetaRegErr = currPosTheta - currTheta;
        double dThetaNegErr = currPosNegTheta - currTheta;
        double dThetaPosErr = currPosPosTheta - currTheta;
        double tarTheta = 0;
        if(Math.abs(dThetaRegErr) > Math.abs(dThetaPosErr) && Math.abs(dThetaNegErr) > Math.abs(dThetaPosErr)){
            tarTheta = currPosPosTheta;
        }else if(Math.abs(dThetaRegErr) > Math.abs(dThetaNegErr) && Math.abs(dThetaPosErr) > Math.abs(dThetaNegErr)){
            tarTheta = currPosNegTheta;
        }else{
            tarTheta = currPosTheta;
        }
        

        SmartDashboard.putNumber("currPos", tarTheta);

        double dTheta = tarTheta - currTheta;

        double omega = 0;

        //NOTE tarTheta IS IN DEGREES BUT OUTPUT WILL BE IN RAD/SEC
        //This part sets the omega based on how far we are from the desired theta
        if(Math.abs(dTheta) < Constants.thresholdT1){
            omega = Constants.tVel1;
        }else if(Math.abs(dTheta) <Constants.thresholdT2){
            omega = Constants.tVel2;
        }else if(Math.abs(dTheta) < Constants.thresholdT3){
            omega = Constants.tVel3;
        }else{
            omega = Constants.tVelOutside;
        }

        //THE SIGNS MIGHT NEED TO BE FLIPPED
        //THIS PART OF THE CODE ADJUSTS THE DIRECTION OF THE OMEGA SO THAT IT GOES THE CORRECT DIRECTION BASED ON WHETHER OUR ERROR IS POSITIVE OR NEGATIVE
         if(dTheta < 0){
             omega = -1 * omega;
          }
        
            drive.velX = velX;
            drive.velY = velY;
            drive.omega = omega;

            //INDEXING STUF
            if(mag < listOfCoords.get(currTarIndex).getTolerance()){
                currTarIndex++;
            }
            if(currTarIndex == listOfCoords.size()){
                isFinish = true;
            }
        
    }
    
    @Override
    public void end(boolean interrupte) {      
        System.out.println("finished com");
        drive.omega = 0;
        drive.velX = 0;
        drive.velY = 0;
    }

    public static int getCurrIndexInner(){
        return currIndexInner;
    }
}
