package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double shooterVelocityLT;
        public double shooterVelocityRT;
        // public double shooterTorqueCurrentLT;
        // public double shooterTorqueCurrentRT;
        // public double shooterDutyCycleLT;
        // public double shooterDutyCycleRT;

        public boolean shooterAdjustBeamBreakState;
        public boolean shooterLoadBeamBreakState;

        public double loadMotorEncCnts;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setShooterEnabled(double velocityLT, double velocityRT) {}

    public default void setShooterEnabled_Hoard() {}

    public default void setShooterDisabled() {}

    public default void toggleHoardVelocityThreshold(boolean toggle) {}

    public default void loadBackward() {}

    public default void fineAdjustBck() {}

    public default void loadDisabled() {}

    public default void loadNote() {}

    public default void feedShooter() {}
    
    public default void fineAdjustFwd() {}

    public default void setServoPosition(double position) {}

    public default void setServoRetract() {}
    
    public default void setServoAngle(double angle) {}
    
    public default void setServoSpeed(double speed) {}

    public default void handoffProcedure() {}

    public default void updateTurretState(){}

    public default void resetLoadEnc(){}

    public default void fineTransferAdjust() {}

}
