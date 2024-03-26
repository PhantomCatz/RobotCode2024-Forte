package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class VisionIOLimeLight implements VisionIO {
    
    public String name;
    public boolean getTarget;
    private double[] lastData = new double[6];

     /**
     * Implements Limelight camera
     *
     * @param name Name of the limelight used, and should be configured in limelight software first
     * @param cameraOffset Location of the camera on the robot (from center, positive x towards the arm, positive y to the left, and positive angle is counterclockwise.
     */
    public VisionIOLimeLight(String name) {
        NetworkTableInstance.getDefault().getTable(name).getEntry("ledMode").setNumber(1);
        this.name = name;
        // System.out.println(name);
        // System.out.println(NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue"));
        
        Logger.recordOutput("Obometry/VisionPose", new Pose2d());

    }

    private Pose2d prevPos = null;
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

        // collects pose information based off network tables and orients itself depending on alliance sid
        //creating new pose3d object based of pose from network tables
        Pose3d pose = llresults.targetingResults.getBotPose3d_wpiBlue();
        inputs.tagCount = llresults.targetingResults.targets_Fiducials.length;

        // set if the Limelight has a target to loggable boolean
        if (inputs.tv == 1) {
            inputs.hasTarget = true;
        } 
        else {
            inputs.hasTarget = false;
        }

        // calculates total latency using 7th table item in array //TBD be more explicit about what latency value gives
        double latency = (llresults.targetingResults.latency_capture + llresults.targetingResults.latency_pipeline) / 1000; //data[6] or latency is recorded in ms; divide by 1000 to get s
        inputs.latency = latency;
        //shoves in new pose2d from pose3d object estimate depending on if new apriltag detected
        if (inputs.hasTarget) {
            // sets input timestamp
            inputs.timestamp = Timer.getFPGATimestamp() - latency;

            inputs.isNewVisionPose = true;

            
            Pose2d pose2d = pose.toPose2d();
            
            if(prevPos == null){
                prevPos = pose.toPose2d();
            }

            if(pose2d.getTranslation().getDistance(prevPos.getTranslation()) > 0.3){
                badData = true;
                pose2d = SubsystemCatzDrivetrain.getInstance().getPose();
            }

            if(!badData){
                prevPos = pose2d;
            }

            badData = false;

            //data used for pose estimator
            inputs.x = pose2d.getX();
            inputs.y = pose2d.getY();
            inputs.rotation = pose2d.getRotation().getRadians();
            Logger.recordOutput("Obometry/VisionPose", new Pose2d(inputs.x,inputs.y,Rotation2d.fromRadians(inputs.rotation)));
        } 
        else {
            inputs.isNewVisionPose = false;
        }

    } 

    @Override
    public String getName() {
        return name;
    }
}