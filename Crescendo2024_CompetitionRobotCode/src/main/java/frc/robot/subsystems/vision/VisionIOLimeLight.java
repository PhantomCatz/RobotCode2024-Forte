package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionIOLimeLight implements VisionIO {
    
    private String name;
    private Transform3d cameraOffset;
    private double[] lastData = new double[6];

     /**
     * Implements Limelight camera
     *
     * @param name Name of the limelight used, and should be configured in limelight software first
     * @param cameraOffset Location of the camera on the robot (from center, positive x towards the arm, positive y to the left, and positive angle is counterclockwise.
     */
    public VisionIOLimeLight(String name, Transform3d limelightOffset) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        this.name = name;
        this.cameraOffset = limelightOffset;
        System.out.println(name);
        System.out.println(NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue"));
        
        //debug for ensuring the limelight is communicating properly with networktables
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
            } catch (Exception e) {
            }
        }).start();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        boolean isAllianceBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
        boolean isAllianceRed  = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

        // collects pose information based off network tables and orients itself depending on alliance side
        NetworkTableEntry botposeEntry;
        if (isAllianceBlue) {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue");
        } 
        else if (isAllianceRed) {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpired");
        } 
        else {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose");
        }

        //logging
        Logger.recordOutput("Vision/AllianceColorBlue", isAllianceBlue);
        Logger.recordOutput("Vision/AllianceColorRed", isAllianceRed);
        
        //creating new pose3d object based of pose from network tables
        double[] data = botposeEntry.getDoubleArray(new double[7]);
        long updateTime = botposeEntry.getLastChange();
        Pose3d pose = new Pose3d(
                data[0], //x translational component
                data[1], //y translational component
                data[2], //z translational component
                new Rotation3d(
                        Math.toRadians(data[3]),   //apriltag roll component
                        Math.toRadians(data[4]),   //apriltag pitch componenet
                        Math.toRadians(data[5])))  //apriltag yaw component
                                        .transformBy(cameraOffset); //apply the camera offset

        // set if the Limelight has a target to loggable boolean
        if (NetworkTableInstance.getDefault().getTable(name).getEntry("tv").getDouble(0) == 1) {
            inputs.hasTarget = true;
            System.out.println("Vision?");
        } 
        else {
            inputs.hasTarget = false;
        }

        // calculates total latency using 7th table item in array
        double latency = data[6] / 1000;

        //shoves in new pose2d from pose3d object estimate depending on if new apriltag detected
        if (inputs.hasTarget) {
            // sets input timestamp
            inputs.timestamp = Logger.getRealTimestamp() - latency;

            inputs.isNewVisionPose = true;

            Pose2d pose2d = pose.toPose2d();

            //data used for pose estimator
            inputs.x = pose2d.getX();
            inputs.y = pose2d.getY();
            inputs.rotation = pose2d.getRotation().getRadians();
        } 
        else {
            inputs.isNewVisionPose = false;
        }

        lastData = data;
    }

    @Override
    public String getName() {
        return name;
    }
}
