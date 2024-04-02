package frc.robot.subsystems.vision;
    
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

 
/*
    Assume the Limelight is the front of the robot
*/
public class SubsystemCatzVision extends SubsystemBase {

    private static SubsystemCatzVision instance = null;

    //io block
    private final VisionIO[] cameras;
    public final VisionIOInputsAutoLogged[] inputs;

    private final List<SubsystemCatzVision.PoseAndTimestamp> results = new ArrayList<>(); //in a list to account for multiple cameras

    private double targetID;
    private int acceptableTagID;
    private boolean useSingleTag = false;

    //constructor for vision subsystem that creates new vision input objects for each camera set in the singleton implementation
    private SubsystemCatzVision(VisionIO[] cameras) {
        this.cameras = cameras;
        inputs = new VisionIOInputsAutoLogged[cameras.length];

        for(int i = 0; i < cameras.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

    }

    /**
    * singleton implenentation of vision
    * Any new cameras should be declared here
    */
    public static SubsystemCatzVision getInstance() {
        if(instance == null) {
            instance = new SubsystemCatzVision(new VisionIO[] {
                new VisionIOLimeLight("limelight-ramen"),   //index 0
                new VisionIOLimeLight("limelight-udon")     //index 1
            });
        }
        return instance;
    }

    @Override
    public void periodic() {

        // clear results from last periodic
        results.clear();
        
        //for every limlight camera process vision with according logic
        for (int i = 0; i < inputs.length; i++) {
            // update and process new inputs[cameraNum] for camera
            cameras[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + cameras[i].getName() + "/Inputs", inputs[i]);
                    
            //checks for when to process vision
            if (inputs[i].hasTarget && 
                inputs[i].isNewVisionPose &&  
                inputs[i].maxDistance < VisionConstants.LOWEST_DISTANCE) { //TBD get rid of this?
                processVision(i);
       
            }
        }        

        //Logging
        Logger.recordOutput("Vision/ResultCount", results.size());
    }

    static int camNum;
    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs[cameraNum

        Pose2d currentPose = new Pose2d(inputs[cameraNum].x, 
                                        inputs[cameraNum].y, 
                                        new Rotation2d(inputs[cameraNum].rotation));

        // add the new pose to a list
        results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp, inputs[cameraNum].tagCount, inputs[cameraNum].ta, cameras[cameraNum].getName()));
        camNum = cameraNum;
    }

    //Returns the last recorded pose in a list
    public List<SubsystemCatzVision.PoseAndTimestamp> getVisionOdometry() {
        return results;
    }

    //Inner class to record a pose and its timestamp
    public static class PoseAndTimestamp {
        private Pose2d pose;
        private double timestamp;
        private int numOfTagsVisible;
        private double avgArea;
        private String name;

        public PoseAndTimestamp(Pose2d pose, double timestamp, int numOfTagsVisible, double avgArea, String name) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.numOfTagsVisible = numOfTagsVisible;
            this.avgArea = avgArea;
            this.name = name;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public int getNumOfTagsVisible(){
            return numOfTagsVisible;
        }

        public double getAvgArea(){
            return avgArea;
        }

        public String getName(){
            return name;
        }
    }

    //------------------------------------------------------------------------
    // Util
    //------------------------------------------------------------------------
    public void setUseSingleTag(boolean useSingleTag, int acceptableTagID) {
        this.useSingleTag = useSingleTag;
        this.acceptableTagID = acceptableTagID;
    }

    
    public double getOffsetX(int cameraNum) {
        return inputs[cameraNum].tx;
    }

    public double getOffsetY(int cameraNum) {
        return inputs[cameraNum].ty;
    }

    public double getAprilTagID(int cameraNum) {
        return inputs[cameraNum].primaryApriltagID;
    }

    public int getCameraNum() {
        return camNum;
    }


}