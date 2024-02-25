package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;

public interface VisionIO {
    
    @AutoLog
    public class VisionIOInputs {
        public double x; //new x pose estimator coordinate value based off limelight
        public double y; //new y pose estimator coordinate value based off limelight
        public double rotation; //new rotation pose estimator coordinate value based off limelight
        public double timestamp;
        public boolean isNewVisionPose; // is it a new pose from a new apriltag estimate

        public double maxAmbiguity;
        public double maxDistance;
        public double minDistance;

        public boolean hasTarget = false;
        public int singleIDUsed;
        public double singleIDUsedDouble;

        public double translationToTargetX;
        public double translationToTargetY;

        public boolean isLimelightCommunicatingWithNetworkTables;

        public double primaryApriltagID; //closest apirltag id that the limelight is communicating with

        public double ty; //vertical offset from crosshair to target in degrees
        public double tx; //horizontal offset from crosshair to target
        public double tv; //whether the limelight has any vaild targets
        public double ta; //target area of the limelight from 0%-100%...how much does the apirltage take up on the frame


    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default String getName() {
        return "";
    }

    public default void setReferencePose(Pose2d pose) {}

    public default void getHorizontalAngle() {}
    
}
