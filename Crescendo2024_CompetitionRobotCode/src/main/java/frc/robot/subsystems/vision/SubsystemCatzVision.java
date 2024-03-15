package frc.robot.subsystems.vision;
    
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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

    //All Apriltag heights are measured from ground level to the center of the Apriltag
    //- Actually all values will be the centimeter recorded values because the inches are in fractions :(
    //Apriltag height values converted from inches

    final double LIMELIGHT_PLACEMENT_HEIGHT_METERS = (50.2/100);//Units.feetToMeters(1.0); 

    //CENTIMETERS 
    final double SOURCE_APRILTAG_HEIGHT_METERS = (130.5/100);//Units.feetToMeters(4.01); 
    final double SPEAKER_APRILTAG_HEIGHT_METERS = (140.5/100);//Units.feetToMeters(4.33071);
    final double STAGE_APRILTAG_HEIGHT_METERS = (129.5/100);//Units.feetToMeters(3.9583);
    final double AMP_APRILTAG_HEIGHT_METERS = (130.5/100);//Units.feetToMeters(4.01);
    final double SPEAKER_HOOD_HEIGHT_METERS = (202/100);
    
    static double aprilTagDistanceToWall;
    static double aprilTagDistanceToSource;
    static double aprilTagDistanceToTrap;
    static double aprilTagDistanceToSpeaker;
    static double aprilTagDistanceToAmp;
    static double distanceToAprilTag;
    static String primaryAprilTag;
    static boolean horizontallyAllignedWithAprilTag;

    double targetID;
    static double horizontalTargetOffset;

    private int acceptableTagID;
    private boolean useSingleTag = false;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tid = table.getEntry("tid");

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
                new VisionIOLimeLight("limelight-udon"),
                new VisionIOLimeLight("limelight-ramen")
            });
        }
        return instance;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("useSingleTag", useSingleTag); //set by driverstation

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
                !DriverStation.isAutonomous() && 
                inputs[i].maxDistance < VisionConstants.LOWEST_DISTANCE) {
                useSingleTag = false;
                if (useSingleTag) {
                    if (inputs[i].singleIDUsed == acceptableTagID) {
                        processVision(i);
                    }
                } 
                else {
                    processVision(i);
                }
            }
        }

        // limelightRangeFinder(1);
        

        //Logging
        Logger.recordOutput("Vision/ResultCount", results.size());

        //log data
        Logger.recordOutput("AprilTagID", primaryAprilTag);
        Logger.recordOutput("Vertical Degrees to Apriltag", inputs[0].ty);
        Logger.recordOutput("Distance to Apriltag", distanceToAprilTag);
        Logger.recordOutput("Distance to Wall", aprilTagDistanceToWall);
    }

    static int camNum;
    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs[cameraNum

        Pose2d currentPose = new Pose2d(inputs[cameraNum].x, 
                                        inputs[cameraNum].y, 
                                        new Rotation2d(inputs[cameraNum].rotation));

        // add the new pose to a list
        results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp, inputs[cameraNum].tagCount, inputs[cameraNum].ta));
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

        public PoseAndTimestamp(Pose2d pose, double timestamp, int numOfTagsVisible, double avgArea) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.numOfTagsVisible = numOfTagsVisible;
            this.avgArea = avgArea;
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

    public double getAprilTagID(int cameraNum) {
        return inputs[cameraNum].primaryApriltagID;
    }

    public int getCameraNum() {
        return camNum;
    }

    //----------------------------------Calculation methods-------------------------------------------

    // public void limelightRangeFinder(int cameraNum) {
    //     if(inputs[cameraNum].primaryApriltagID == 1 || 
    //        inputs[cameraNum].primaryApriltagID == 2 || 
    //        inputs[cameraNum].primaryApriltagID == 9 || 
    //        inputs[cameraNum].primaryApriltagID == 10) 
    //     {
    //         //Source
    //         primaryAprilTag = "Source";

    //         //vertical distance to target
    //         distanceToAprilTag = (SOURCE_APRILTAG_HEIGHT_METERS - LIMELIGHT_PLACEMENT_HEIGHT_METERS) / Math.sin(inputs[cameraNum].ty);
    //         aprilTagDistanceToWall = (SOURCE_APRILTAG_HEIGHT_METERS - LIMELIGHT_PLACEMENT_HEIGHT_METERS) / Math.tan(inputs[cameraNum].ty);

    //         //horizontal distance to target
    //         horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(inputs[cameraNum].tx);
            

    //         if(Math.abs(horizontalTargetOffset) < 5) //Distance Target %
    //         {
    //             System.out.println("Alligned with Target");
    //             horizontallyAllignedWithAprilTag = true;

    //             NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

    //         } else {
    //             NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    //         }
    //     } 
    //     else if (inputs[cameraNum].primaryApriltagID == 3 || 
    //              inputs[cameraNum].primaryApriltagID == 4 || 
    //              inputs[cameraNum].primaryApriltagID == 7 || 
    //              inputs[cameraNum].primaryApriltagID == 8)  
    //     {
    //         //Speaker
    //         primaryAprilTag = "Speaker";

    //         //vertical distance to target
    //         distanceToAprilTag = (SPEAKER_APRILTAG_HEIGHT_METERS - LIMELIGHT_PLACEMENT_HEIGHT_METERS) / Math.sin(inputs[cameraNum].ty);
    //         aprilTagDistanceToWall = (SPEAKER_APRILTAG_HEIGHT_METERS - LIMELIGHT_PLACEMENT_HEIGHT_METERS) / Math.tan(inputs[cameraNum].ty);
        
    //         //horizontal distance to target
    //         horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(inputs[cameraNum].tx);

    //         if(horizontalTargetOffset > 5 && horizontalTargetOffset < 5) 
    //         {
    //             System.out.println("Alligned with Target");
    //             horizontallyAllignedWithAprilTag = true;

    //             NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

    //         } else {
    //             NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    //         }
    //     } 
    //     else if (inputs[cameraNum].primaryApriltagID == 11 || 
    //              inputs[cameraNum].primaryApriltagID == 12 || 
    //              inputs[cameraNum].primaryApriltagID == 13 || 
    //              inputs[cameraNum].primaryApriltagID == 14 || 
    //              inputs[cameraNum].primaryApriltagID == 15 || 
    //              inputs[cameraNum].primaryApriltagID == 16) 
    //     {
    //         //Trap
    //         primaryAprilTag = "Trap";

    //         //vertical distance to target
    //         distanceToAprilTag = (STAGE_APRILTAG_HEIGHT_METERS - LIMELIGHT_PLACEMENT_HEIGHT_METERS) / Math.sin(inputs[cameraNum].ty);
    //         aprilTagDistanceToWall = (STAGE_APRILTAG_HEIGHT_METERS - LIMELIGHT_PLACEMENT_HEIGHT_METERS) / Math.tan(inputs[cameraNum].ty);
            
    //         //horizontal distance to target
    //         horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(inputs[cameraNum].tx);  
    //         if(horizontalTargetOffset > 5 && horizontalTargetOffset < 5) 
    //         {
    //             System.out.println("Alligned with Target");
    //             horizontallyAllignedWithAprilTag = true;

    //             NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

    //         } else {
    //             NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    //         }       
    //     } 
    //     else if (inputs[cameraNum].primaryApriltagID == 5 || 
    //              inputs[cameraNum].primaryApriltagID == 6) 
    //     {
    //         //Amp
    //         primaryAprilTag = "Amp";

    //         //vertical distance to target
    //         distanceToAprilTag = (AMP_APRILTAG_HEIGHT_METERS - LIMELIGHT_PLACEMENT_HEIGHT_METERS) / Math.sin(inputs[cameraNum].ty);
    //         aprilTagDistanceToWall = (AMP_APRILTAG_HEIGHT_METERS - LIMELIGHT_PLACEMENT_HEIGHT_METERS) / Math.tan(inputs[cameraNum].ty);

    //         //horizontal distance to target
    //         horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(inputs[cameraNum].tx);
    //         if(horizontalTargetOffset > 5 && horizontalTargetOffset < 5) 
    //         {
    //             System.out.println("Alligned with Target");
    //             horizontallyAllignedWithAprilTag = true;

    //             NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

    //         } else {
    //             NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    //         }
    //     }   
    // } 


}