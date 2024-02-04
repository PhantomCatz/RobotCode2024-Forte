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
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;


/*
    Assume the Limelight is the front of the robot
*/
public class SubsystemCatzVision extends SubsystemBase {

    final double limelightPlacementHeight = Units.feetToMeters(1.8);

    static double angleAdjustment;
    static double speakerHoodDistance;

    static double aprilTagDistanceToWall;
    static double aprilTagDistanceToSource;
    static double aprilTagDistanceToTrap;
    static double aprilTagDistanceToSpeaker;
    static double aprilTagDistanceToAmp;
    static double distanceToAprilTag;
    static String primaryAprilTag;
    static double shooterHoodAngle;

    static boolean horizontallyAllignedWithAprilTag;
    static boolean aprilTagInView;
    static String fieldSide;

    final double testHeight = 11.5;
    static double wallDistance;
    static double aprilTagDistance;

    String name;

    static double horizontalTargetOffset;

    private static SubsystemCatzVision instance = null;

    private final VisionIO[] cameraArrayio;
    private final VisionIOInputsAutoLogged[] inputs;

    private final List<SubsystemCatzVision.PoseAndTimestamp> results = new ArrayList<>(); //in a list to account for multiple cameras

    private int acceptableTagID;
    private boolean useSingleTag = false;

    //constructor for vision subsystem that creates new vision input objects for each camera set in the singleton implementation
    private SubsystemCatzVision(VisionIO[] cameras) {
        this.cameraArrayio = cameras;
        inputs = new VisionIOInputsAutoLogged[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
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
            // update and process new inputs for camera
            cameraArrayio[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + cameraArrayio[i].getName() + "/Inputs", inputs[i]);
            
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

        //Logging
        Logger.recordOutput("Vision/ResultCount", results.size());

        // //for every limlight camera process vision with according logic
        //     // update and process new inputs for camera
        //     cameraArrayio.updateInputs(inputs);
        //     Logger.processInputs("Vision/" + cameraArrayio.getName() + "/Inputs", inputs);
                    
        //     //checks for when to process vision
        //     if (inputs.hasTarget && 
        //         inputs.isNewVisionPose && 
        //         !DriverStation.isAutonomous() && 
        //         inputs.maxDistance < VisionConstants.LOWEST_DISTANCE) {
        //         if (useSingleTag) {
        //             if (inputs.singleIDUsed == acceptableTagID) {
        //                 processVision();
        //             }
        //         } 
        //         else {
        //             processVision();
        //         }
        //     }
        limelightRangeFinder();
        speakerHoodAdjustment();

        //  TOP DOWN PERSPECTIVE (Speaker)
        //
        //                   A   <- (Apriltag)    
        //                 / |
        //                /  | 
        //               /   |
        //              /    | (x)
        //             /     |
        //            / 0x   |  
        //          |o|______|                  A = Apriltag
        //           |   (z)                    z = horizontal target offset
        //        (Robot)                       x = distance to wall
        //                                     0x = horizontal degrees to Apriltag

        //  SIDE PERSPECTIVE (Speaker)
        //                        _ _ 
        //                       /  |  <- (Speaker Hood)
        //                      |___|  _
        //                          |  _|(y2)
        //                        __A  _     <-- (Apriltag)
        //                     __/  |   |  
        //             (dy) __/     |   |(y1)  
        //               __/        |   |
        //              /0y         |   |       A = Apriltag
        //          |T=T|___________|  _|       x = distance to wall
        //           ||       (x)              y1 = vertical distance till Apriltag
        //        (Robot)                      y2 = vertical distance inbetween Apriltag and Speaker Hood
        //                                     dy = vertical distance to Apriltag
        //                                     0y = vertical degreees to Apriltag

        //Logging
        Logger.recordOutput("Vision/ResultCount", results.size());

        //log data
        Logger.recordOutput("AprilTagID", primaryAprilTag);
        Logger.recordOutput("Target Alligned?", horizontallyAllignedWithAprilTag);

        //Hoz and Ver Angles from limelight crosshair
        Logger.recordOutput("Vertical Degrees", inputs[0].ty);   //Vertical Angle
        Logger.recordOutput("Horizontal Degrees", inputs[0].tx); //Horizontal Angle
        
        //Classic Apriltag Ranging
        Logger.recordOutput("AprilTag DIstance", distanceToAprilTag);
        Logger.recordOutput("Distance to Wall", aprilTagDistanceToWall);

        // Z
        Logger.recordOutput("Horizontal Distance", horizontalTargetOffset);

        //Speaker Hood Adjustments
        Logger.recordOutput("angleAdjustment", (angleAdjustment));
        Logger.recordOutput("SpeakerHoodDistance", speakerHoodDistance);
        Logger.recordOutput("ShooterHoodAngle", (shooterHoodAngle));
    }

    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs
        Pose2d currentPose = new Pose2d(inputs[cameraNum].x, 
                                        inputs[cameraNum].y, 
                                        new Rotation2d(inputs[cameraNum].rotation));

        //log data
        Logger.recordOutput(cameraArrayio[cameraNum].getName() + " pose", currentPose);

        // add the new pose to a list
        results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp));
    }

    // public void processVision() {
    //     // create a new pose based off the new inputs
    //     Pose2d currentPose = new Pose2d(inputs.x, 
    //                                     inputs.y, 
    //                                     new Rotation2d(inputs.rotation));

    //     // add the new pose to a list
    //     results.add(new PoseAndTimestamp(currentPose, inputs.timestamp));
    // }

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

    public double getMinDistance() {
         return inputs[0].minDistance;
    }

    //----------------------------------------------------------------------Calculation methods---------------------------------------------

    static double stealingAngle;
    static double stealingPrimaryTag;
    static double stealingHozAngle;
    static double stealingHozDistance;
    static double stealingVerDistance;

    public void TF2() {
        if(getPrimaryAprilTag() == 1  ||
           getPrimaryAprilTag() == 2  ||
           getPrimaryAprilTag() == 3  ||
           getPrimaryAprilTag() == 4  ||
           getPrimaryAprilTag() == 5  ||
           getPrimaryAprilTag() == 11 ||
           getPrimaryAprilTag() == 12 ||
           getPrimaryAprilTag() == 13) {
            aprilTagInView = true;
            fieldSide = "Red";
        } else if (getPrimaryAprilTag() == 6  ||
                   getPrimaryAprilTag() == 7  ||
                   getPrimaryAprilTag() == 8  ||
                   getPrimaryAprilTag() == 9  ||
                   getPrimaryAprilTag() == 10 ||
                   getPrimaryAprilTag() == 14 ||
                   getPrimaryAprilTag() == 15 ||
                   getPrimaryAprilTag() == 16) {
            aprilTagInView = true;
            fieldSide = "Blue";
        } else {
            aprilTagInView = false;
        }
    }
    public static double speakerHoodAdjustment() {
        double shooterAngle;
        if(getPrimaryAprilTag() == 3  ||
           getPrimaryAprilTag() == 4  ||
           getPrimaryAprilTag() == 7  ||
           getPrimaryAprilTag() == 8) {
            
          angleAdjustment = Units.radiansToDegrees(Math.atan((VisionConstants.SPEAKER_HOOD_HEIGHT - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / aprilTagDistanceToWall)) - getAngle();
          speakerHoodDistance = (VisionConstants.SPEAKER_HOOD_HEIGHT - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.sin(Units.degreesToRadians(getAngle() + angleAdjustment));
        } 
        shooterAngle = angleAdjustment + getAngle();
    
        return shooterAngle;
    }

    private void limelightRangeFinder() {
    //determine what apriltag feild station robot is facing
        if(inputs[1].primaryApriltagID == 1 || 
           inputs[1].primaryApriltagID == 2 || 
           inputs[1].primaryApriltagID == 9 || 
           inputs[1].primaryApriltagID == 10) {
            //Source
            primaryAprilTag = "Source";

            //vertical distance to target
            distanceToAprilTag = (VisionConstants.sourceApriltagHeight - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.sin(Units.degreesToRadians(inputs[1].ty)); 
            aprilTagDistanceToWall = (VisionConstants.sourceApriltagHeight - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.tan(Units.degreesToRadians(inputs[1].ty));

            //horizontal distance to target
            horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(Units.degreesToRadians(inputs[1].tx));
            if(horizontalTargetOffset > -Units.feetToMeters(1) && horizontalTargetOffset < Units.feetToMeters(1)) {
                System.out.println("Alligned with Target");
                horizontallyAllignedWithAprilTag = true;

                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2); //blink

            } else {
                horizontallyAllignedWithAprilTag = false;
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //off
            }
        } 
        else if (inputs[0].primaryApriltagID == 3 || 
                 inputs[0].primaryApriltagID == 4 || 
                 inputs[0].primaryApriltagID == 7 || 
                 inputs[0].primaryApriltagID == 8) {
            //Speaker
            primaryAprilTag = "Speaker";

            //vertical distance to target
            distanceToAprilTag = (VisionConstants.SPEAKER_APRILTAG_HEIGHT - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.sin(Units.degreesToRadians(inputs[0].ty - 4)); 
            aprilTagDistanceToWall = (VisionConstants.SPEAKER_APRILTAG_HEIGHT - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.tan(Units.degreesToRadians(inputs[0].ty - 4));
        
            //horizontal distance to target
            horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(Units.degreesToRadians(inputs[0].tx));

            if(horizontalTargetOffset > -Units.feetToMeters(1) && horizontalTargetOffset < Units.feetToMeters(1)) {// 5 what?? I don't know
                System.out.println("Alligned with Target");
                horizontallyAllignedWithAprilTag = true;

                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

            } else {
                horizontallyAllignedWithAprilTag = false;
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }
        } 
        else if (inputs[1].primaryApriltagID == 11 || 
                 inputs[1].primaryApriltagID == 12 || 
                 inputs[1].primaryApriltagID == 13 || 
                 inputs[1].primaryApriltagID == 14 || 
                 inputs[1].primaryApriltagID == 15 || 
                 inputs[1].primaryApriltagID == 16) {
            //Trap
            primaryAprilTag = "Trap";

            //vertical distance to target
            distanceToAprilTag = (VisionConstants.trapApriltagHeight - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.sin(Units.degreesToRadians(inputs[1].ty));
            aprilTagDistanceToWall = (VisionConstants.trapApriltagHeight - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.tan(Units.degreesToRadians(inputs[1].ty));
            
            //horizontal distance to target
            horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(Units.degreesToRadians(inputs[1].tx));  
            if(horizontalTargetOffset > -Units.feetToMeters(1) && horizontalTargetOffset < Units.feetToMeters(1)) {// 5 what?? I don't know
                System.out.println("Alligned with Target");
                horizontallyAllignedWithAprilTag = true;

                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

            } else {
                horizontallyAllignedWithAprilTag = false;
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }       
        } 
        else if (inputs[1].primaryApriltagID == 5 || 
                 inputs[1].primaryApriltagID == 6) {
            //Amp
            primaryAprilTag = "Amp";

            //vertical distance to target
            distanceToAprilTag = (VisionConstants.ampApriltagHeight - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.sin(Units.degreesToRadians(inputs[1].ty));
            aprilTagDistanceToWall = (VisionConstants.ampApriltagHeight - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.tan(Units.degreesToRadians(inputs[1].ty));

            //horizontal distance to target
            horizontalTargetOffset = (aprilTagDistanceToWall) * Math.tan(Units.degreesToRadians(inputs[1].tx));
            if(horizontalTargetOffset > -Units.feetToMeters(1) && horizontalTargetOffset < Units.feetToMeters(1)) { // 5 what?? I don't know
                System.out.println("Alligned with Target");
                horizontallyAllignedWithAprilTag = true;

                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

            } else {
                horizontallyAllignedWithAprilTag = false;
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }
        }   
        stealingAngle = Units.degreesToRadians(inputs[1].ty);
        stealingPrimaryTag = inputs[1].primaryApriltagID;
        stealingHozAngle = Units.degreesToRadians(inputs[1].tx);
        stealingHozDistance = horizontalTargetOffset;
        stealingVerDistance = VisionConstants.SPEAKER_APRILTAG_HEIGHT - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT;

        if(getPrimaryAprilTag() == 3  ||
           getPrimaryAprilTag() == 4  ||
           getPrimaryAprilTag() == 7  ||
           getPrimaryAprilTag() == 8) {
            angleAdjustment = Math.tanh((VisionConstants.SPEAKER_APRILTAG_HEIGHT + VisionConstants.HOOD_TO_APRILTAG - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / aprilTagDistanceToWall);
            speakerHoodDistance = (VisionConstants.SPEAKER_APRILTAG_HEIGHT + VisionConstants.HOOD_TO_APRILTAG - VisionConstants.LIMELIGHT_PLACEMENT_HEIGHT) / Math.sin((inputs[0].ty + angleAdjustment)); //SubsystemCatzVision.getAngle() + angleAdjustment));
        } 
        shooterHoodAngle = angleAdjustment + inputs[0].ty;
    } 

    public static double getApriltagToWall() {
        return aprilTagDistanceToWall;
    }

    public static double getAngle() {
        return stealingAngle - 4;
    }

    public static double getPrimaryAprilTag() {
        return stealingPrimaryTag;
    } 

    public double getHorizontalAngle() {
        return stealingHozAngle;
    }

    public static double getHorizontalDistance() {
        return stealingHozDistance;
    }

    public static double getVerticalDistance() {
        return stealingVerDistance;
    }

    public void updatePose() {
        if(primaryAprilTag == "Speaker") {
            
        }
    }

    // Pose2d a1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    // Pose2d a2 = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
    // Pose2d a3 = new Pose2d(2, 0, Rotation2d.fromDegrees(0));
    // Pose2d b1 = new Pose2d(0, 1, Rotation2d.fromDegrees(0));

    static Timer time = new Timer();
    static double X;
    static double Y;
    static double changeOfX;
    static double changeOfY;
    static double degrees;
    public void startAutoTarget() {
      time.reset();
      time.start();
      double Time = time.get();
  
      Pose2d start = new Pose2d(SubsystemCatzDrivetrain.getInstance().getPose().getX(), 
                                SubsystemCatzDrivetrain.getInstance().getPose().getY(), 
                                SubsystemCatzDrivetrain.getInstance().getRotation2d());
  
      X = SubsystemCatzDrivetrain.getInstance().getPose().getX();
      Y = SubsystemCatzVision.getApriltagToWall();
  
      changeOfX = X / Time;
      changeOfY = Y / Time;
  
      degrees = (((Y * changeOfX) - (X * changeOfY)) / (1 + ((X / Y) * (X / Y))));
    }    


    /**
    * singleton implenentation of vision
    * Any new cameras should be declared here
    */
    public static SubsystemCatzVision getInstance() {
        if(instance == null) {
            // instance = new SubsystemCatzVision(
            //     new VisionIOLimeLight("limelight", VisionConstants.LIMELIGHT_OFFSET)
            //     );
            instance = new SubsystemCatzVision(new VisionIO[] {
                new VisionIOLimeLight("limelight", VisionConstants.LIMELIGHT_OFFSET)
            });
        }
        return instance;
    }
    
    

}
