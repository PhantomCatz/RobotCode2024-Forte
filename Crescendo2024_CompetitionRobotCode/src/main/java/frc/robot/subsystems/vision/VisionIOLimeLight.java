package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.FieldConstants;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class VisionIOLimeLight implements VisionIO {
    
    public String name;
    public boolean getTarget;
    private double[] lastData = new double[6];

    private int primaryTrackingApriltag;

     /**
     * Implements Limelight camera
     *
     * @param name Name of the limelight used, and should be configured in limelight software first
     */
    public VisionIOLimeLight(String name) {
        NetworkTableInstance.getDefault().getTable(name).getEntry("ledMode").setNumber(1);
        this.name = name;
        System.out.println("Limeilight " + name + " instantiated");
        
    }

    private Pose2d prevVisionPos = null;
    private Pose2d visionPose2d = null;
    private boolean badData = false;

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        //load up raw apriltag values for distance calculations
        LimelightResults llresults = LimelightHelpers.getLatestResults(name);
        inputs.ty = NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0); //vertical offset from crosshair to target in degrees
        inputs.tx = NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0); //horizontal offset from crosshair to target
        inputs.tv = NetworkTableInstance.getDefault().getTable(name).getEntry("tv").getDouble(0); //whether the limelight has any vaild targets
        inputs.ta = NetworkTableInstance.getDefault().getTable(name).getEntry("ta").getDouble(0); //target area of the limelight from 0%-100%...how much does the apirltage take up on the frame
        inputs.primaryApriltagID = NetworkTableInstance.getDefault().getTable(name).getEntry("tid").getDouble(0);

        // collects pose information based off network tables and orients itself depending on alliance side
        //creating new pose3d object based of pose from network tables
        double[] visionPoseInfo = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        inputs.tagCount = llresults.targetingResults.targets_Fiducials.length;
        inputs.maxDistance = llresults.targetingResults.botpose_avgdist;

        // set if the Limelight has a target to loggable boolean
        if (inputs.tv == 1) {
            inputs.hasTarget = true;
        } 
        else {
            inputs.hasTarget = false;
        }

        // calculates total latency using 7th table item in array 
        double latency = (llresults.targetingResults.latency_capture + llresults.targetingResults.latency_pipeline) / 1000; //data[6] or latency is recorded in ms; divide by 1000 to get s
        inputs.latency = latency;
        //shoves in new pose2d from pose3d object estimate depending on if new apriltag detected
        
        if (inputs.hasTarget) {
            // sets input timestamp
            inputs.timestamp = Timer.getFPGATimestamp() - latency;

            inputs.isNewVisionPose = true;

            
            visionPose2d = new Pose2d(visionPoseInfo[0],visionPoseInfo[1], new Rotation2d());
            if(prevVisionPos == null){
                prevVisionPos = visionPose2d;
            }

            if(visionPose2d.getTranslation().getDistance(prevVisionPos.getTranslation()) > 0.3){
                badData = true;
                visionPose2d = SubsystemCatzDrivetrain.getInstance().getPose();
                prevVisionPos = SubsystemCatzDrivetrain.getInstance().getPose();
            }

            if(!badData){
                prevVisionPos = visionPose2d;
            }

            badData = false;

            //data used for pose estimator
            inputs.x = visionPose2d.getX();
            inputs.y = visionPose2d.getY();
            inputs.rotation = visionPose2d.getRotation().getRadians();
        } 
        else {
            inputs.isNewVisionPose = false;
            visionPose2d = SubsystemCatzDrivetrain.getInstance().getPose();
            prevVisionPos = null;
        }

    } 

    @Override
    public String getName() {
        return name;
    }
}