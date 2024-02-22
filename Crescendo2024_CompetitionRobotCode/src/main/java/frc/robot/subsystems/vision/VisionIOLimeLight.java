package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants.AllianceColor;

public class VisionIOLimeLight implements VisionIO {
    
    public String name;
    private Transform3d cameraOffset;
    private double[] lastData = new double[6];

     /**
     * Implements Limelight camera
     *
     * @param name Name of the limelight used, and should be configured in limelight software first
     * @param cameraOffset Location of the camera on the robot (from center, positive x towards the arm, positive y to the left, and positive angle is counterclockwise.
     */
    public VisionIOLimeLight(String name, Transform3d limelightOffset) {
        NetworkTableInstance.getDefault().getTable(name).getEntry("ledMode").setNumber(1);
        this.name = name;
        this.cameraOffset = limelightOffset;
        
        System.out.println(name);
        System.out.println(NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue"));
        
        
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        //load up raw apriltag values for distance calculations
        inputs.tx = NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0);
        inputs.ty = NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0);
        inputs.tv = NetworkTableInstance.getDefault().getTable(name).getEntry("tv").getDouble(0);
        inputs.ta = NetworkTableInstance.getDefault().getTable(name).getEntry("ta").getDouble(0);
        inputs.primaryApriltagID = NetworkTableInstance.getDefault().getTable(name).getEntry("tid").getDouble(0);


        boolean isAllianceBlue = CatzAutonomous.getInstance().getAllianceColor() == AllianceColor.Blue;

        // collects pose information based off network tables and orients itself depending on alliance side
        NetworkTableEntry botposeEntry;
        if (isAllianceBlue) {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue");
        } 
        else {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpired");
        } 
        // else {
        //     botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose");
        // }

        //logging
        Logger.recordOutput("Vision/IsAllianceColorBlue", isAllianceBlue);
        
        //creating new pose3d object based of pose from network tables
        double[] data = botposeEntry.getDoubleArray(new double[7]);
        Pose3d pose = new Pose3d(
                data[0], //x translational component
                data[1], //y translational component
                data[2], //z translational component
                new Rotation3d(
                        Math.toRadians(data[3]),   //apriltag roll component
                        Math.toRadians(data[4]),   //apriltag pitch componenet
                        Math.toRadians(data[5])));  //apriltag yaw component
                                        //.transformBy(cameraOffset); //apply the camera offset TBD this is breaking the limelight


        // set if the Limelight has a target to loggable boolean
        if (inputs.tv == 1) {
            inputs.hasTarget = true;
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
            inputs.x = pose2d.getX() + cameraOffset.getX();
            inputs.y = pose2d.getY() + cameraOffset.getY();
            inputs.rotation = pose2d.getRotation().getRadians() + cameraOffset.getRotation().getAngle();
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
