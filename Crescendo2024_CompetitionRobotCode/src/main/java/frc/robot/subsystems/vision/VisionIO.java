package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    
    @AutoLog
    public class VisionIOInputs {
        public double x;
        public double y;
        public double rotation;
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
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default String getName() {
        return "";
    }

    public default void setReferencePose(Pose2d pose) {}
}
