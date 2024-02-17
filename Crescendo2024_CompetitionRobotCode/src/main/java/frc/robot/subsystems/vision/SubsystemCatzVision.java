package frc.robot.subsystems.vision;
    
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

    final double limelightPlacementHeight = Units.feetToMeters(1.0);
    final double sourceApriltagHeight = Units.feetToMeters(4.0);
    final double speakerApriltagHeight = Units.feetToMeters(4.33);
    final double trapApriltagHeight = Units.feetToMeters(3.969);
    final double ampApriltagHeight = 1.22;

    static double aprilTagDistanceToWall;
    static double aprilTagDistanceToSource;
    static double aprilTagDistanceToTrap;
    static double aprilTagDistanceToSpeaker;
    static double aprilTagDistanceToAmp;
    static double distanceToAprilTag;
    static String primaryAprilTag;
    static boolean horizontallyAllignedWithAprilTag;

    String name;

    static double horizontalTargetOffset;

    private static SubsystemCatzVision instance = null;

    private final VisionIO[] cameras;
    private final VisionIOInputsAutoLogged[] inputs;

    private final List<SubsystemCatzVision.PoseAndTimestamp> results = new ArrayList<>(); //in a list to account for multiple cameras

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

    //NOTE TO EVERYONE...DON'T GET RID OF UNCOMMETED CODE PLZ (LMAO)

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
            
        System.out.println("inputs processed");
        
            //checks for when to process vision
            if (inputs[i].hasTarget && 
                inputs[i].isNewVisionPose && 
                !DriverStation.isAutonomous() && 
                inputs[i].maxDistance < VisionConstants.LOWEST_DISTANCE) {
                if (useSingleTag) {
                    if (inputs[i].singleIDUsed == acceptableTagID) {
                        processVision(i);
                    }
                } 
                else {
                    processVision(i);
                }
                System.out.println("vision processeed");
            }
        }

        // //Logging
        // Logger.recordOutput("Vision/ResultCount", results.size());

        //for every limlight camera process vision with according logic
            // update and process new inputs[cameraNum] for camera
            // camera.updateInputs(inputs[cameraNum]);
            // Logger.processInputs("Vision/" + camera.getName() + "/Inputs", inputs[cameraNum]);
                    
            // //checks for when to process vision
            // if (inputs[cameraNum].hasTarget && 
            //     inputs[cameraNum].isNewVisionPose && 
            //     !DriverStation.isAutonomous() && 
            //     inputs[cameraNum].maxDistance < VisionConstants.LOWEST_DISTANCE) {
            //     if (useSingleTag) {
            //         if (inputs[cameraNum].singleIDUsed == acceptableTagID) {
            //             processVision();
            //         }
            //     } 
            //     else {
            //         processVision();
            //     }
            // }
        limelightRangeFinder(1);
        

        //Logging
        Logger.recordOutput("Vision/ResultCount", results.size());

        //log data
        Logger.recordOutput("AprilTagID", primaryAprilTag);
        Logger.recordOutput("Vertical Degrees to Apriltag", inputs[1].ty);
        Logger.recordOutput("Distance to Apriltag", distanceToAprilTag);
        Logger.recordOutput("Distance to Wall", aprilTagDistanceToWall);
    }

    // public void processVision(int cameraNum) {
    //     // create a new pose based off the new inputs[cameraNum]
    //     Pose2d currentPose = new Pose2d(inputs[cameraNum][cameraNum].x, 
    //                                     inputs[cameraNum][cameraNum].y, 
    //                                     new Rotation2d(inputs[cameraNum][cameraNum].rotation));

    //     //log data
    //     Logger.recordOutput(cameras[cameraNum].getName() + " pose", currentPose);

    //     // add the new pose to a list
    //     results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum][cameraNum].timestamp));
    // }

    static int camNum;
    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs[cameraNum]
        Pose2d currentPose = new Pose2d(inputs[cameraNum].x, 
                                        inputs[cameraNum].y, 
                                        new Rotation2d(inputs[cameraNum].rotation));

        // add the new pose to a list
        results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp));
        camNum = cameraNum;
    }

    //Returns the last recorded pose in a list
    public List<SubsystemCatzVision.PoseAndTimestamp> getVisionOdometry() {
        return results;
    }

    //Inner class to record a pose and its timestamp
    public static class PoseAndTimestamp {
        Pose2d pose;
        double timestamp;

        public PoseAndTimestamp(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

    //access method for determining whether to use multiple tags for pose estimation
    public void setUseSingleTag(boolean useSingleTag) {
        setUseSingleTag(useSingleTag, 0);
    }

    public void setUseSingleTag(boolean useSingleTag, int acceptableTagID) {
        this.useSingleTag = useSingleTag;
        this.acceptableTagID = acceptableTagID;
    }

    public double getMinDistance(int cameraNum) {
         return inputs[cameraNum].minDistance;
    }

    public double getOffsetX(int cameraNum) {
        return inputs[cameraNum].tx;
    }

    public double getAprilTagID(int cameraNum) {
        return inputs[cameraNum].primaryApriltagID;
    }

    //----------------------------------Calculation methods---------------------------------------------
    
    public void limelightRangeFinder(int cameraNum) {
        if(inputs[cameraNum].primaryApriltagID == 1 || 
           inputs[cameraNum].primaryApriltagID == 2 || 
           inputs[cameraNum].primaryApriltagID == 9 || 
           inputs[cameraNum].primaryApriltagID == 10) 
        {
            //Source
            primaryAprilTag = "Source";

            //vertical distance to target
            distanceToAprilTag = (sourceApriltagHeight - limelightPlacementHeight) / Math.sin(inputs[cameraNum].ty);
            aprilTagDistanceToWall = (sourceApriltagHeight - limelightPlacementHeight) / Math.tan(inputs[cameraNum].ty);

            //horizontal distance to target
            horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(inputs[cameraNum].tx);
            if(Math.abs(horizontalTargetOffset) > 5) // 5 what?? I don't know
            {
                System.out.println("Alligned with Target");
                horizontallyAllignedWithAprilTag = true;

                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

            } else {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }
        } 
        else if (inputs[cameraNum].primaryApriltagID == 3 || 
                 inputs[cameraNum].primaryApriltagID == 4 || 
                 inputs[cameraNum].primaryApriltagID == 7 || 
                 inputs[cameraNum].primaryApriltagID == 8)  
        {
            //Speaker
            primaryAprilTag = "Speaker";

            //vertical distance to target
            distanceToAprilTag = (speakerApriltagHeight - limelightPlacementHeight) / Math.sin(inputs[cameraNum].ty);
            aprilTagDistanceToWall = (speakerApriltagHeight - limelightPlacementHeight) / Math.tan(inputs[cameraNum].ty);
        
            //horizontal distance to target
            horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(inputs[cameraNum].tx);

            if(horizontalTargetOffset > 5 && horizontalTargetOffset < 5) // 5 what?? I don't know
            {
                System.out.println("Alligned with Target");
                horizontallyAllignedWithAprilTag = true;

                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

            } else {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }
        } 
        else if (inputs[cameraNum].primaryApriltagID == 11 || 
                 inputs[cameraNum].primaryApriltagID == 12 || 
                 inputs[cameraNum].primaryApriltagID == 13 || 
                 inputs[cameraNum].primaryApriltagID == 14 || 
                 inputs[cameraNum].primaryApriltagID == 15 || 
                 inputs[cameraNum].primaryApriltagID == 16) 
        {
            //Trap
            primaryAprilTag = "Trap";

            //vertical distance to target
            distanceToAprilTag = (trapApriltagHeight - limelightPlacementHeight) / Math.sin(inputs[cameraNum].ty);
            aprilTagDistanceToWall = (trapApriltagHeight - limelightPlacementHeight) / Math.tan(inputs[cameraNum].ty);
            
            //horizontal distance to target
            horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(inputs[cameraNum].tx);  
            if(horizontalTargetOffset > 5 && horizontalTargetOffset < 5) // 5 what?? I don't know
            {
                System.out.println("Alligned with Target");
                horizontallyAllignedWithAprilTag = true;

                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

            } else {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }       
        } 
        else if (inputs[cameraNum].primaryApriltagID == 5 || 
                 inputs[cameraNum].primaryApriltagID == 6) 
        {
            //Amp
            primaryAprilTag = "Amp";

            //vertical distance to target
            distanceToAprilTag = (ampApriltagHeight - limelightPlacementHeight) / Math.sin(inputs[cameraNum].ty);
            aprilTagDistanceToWall = (ampApriltagHeight - limelightPlacementHeight) / Math.tan(inputs[cameraNum].ty);

            //horizontal distance to target
            horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(inputs[cameraNum].tx);
            if(horizontalTargetOffset > 5 && horizontalTargetOffset < 5) // 5 what?? I don't know
            {
                System.out.println("Alligned with Target");
                horizontallyAllignedWithAprilTag = true;

                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

            } else {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }
        }   
    } 

    public int getCameraNum() {
        return camNum;
    }


    /**
    * singleton implenentation of vision
    * Any new cameras should be declared here
    */
    public static SubsystemCatzVision getInstance() {
        if(instance == null) {
            instance = new SubsystemCatzVision(new VisionIO[] {
                new VisionIOLimeLight("limelight-turret", VisionConstants.LIMELIGHT_TURRET_OFFSET),
                new VisionIOLimeLight("limelight", VisionConstants.LIMELIGHT_OFFSET)
            });
        }
        return instance;
    }

}
