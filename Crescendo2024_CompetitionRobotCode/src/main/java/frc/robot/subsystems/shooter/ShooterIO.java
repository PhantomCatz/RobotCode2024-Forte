package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double shooterVelocityRpsLT;
        public double shooterVelocityRpsRT;
        public double shooterPercentOutputLT;
        public double shooterPercentOutputRT;
        public double shooterMotorVoltageLT;
        public double shooterMotorVoltageRT;
        public double shooterTorqueCurrentLT;
        public double shooterTorqueCurrentRT;

        public double LoadMotorPercentOutput; 
        public double LoadMotorVelocity;
        public double FeedPercentOutput;
        public double FeedVelocity;

        public double shooterVelocityErrorLT;
        public double shooterVelocityErrorRT;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setFlywheelVelocity(double velocityLT, double velocityRT) {}

    public default void setShooterDisabled() {}

    public default void shootLoadPercentOutput(double percentOutput) {}

    public default void setLoadDisabled() {}

    public default void setFeedPercentOuput(double percentOutput) {}
}
