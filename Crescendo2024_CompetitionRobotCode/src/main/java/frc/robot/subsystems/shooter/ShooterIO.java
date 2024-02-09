package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double shooterVelocityLT;
        public double shooterVelocityRT;
        public double shooterDutyCycleLT;
        public double shooterDutyCycleRT;
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

    public default void shootWithVelocity() {}

    public default void setShooterDisabled() {}

    public default void shootFeederReverse() {}

    public default void setFeederDisabled() {}

    public default void shootFeederWithVelocity() {}

    public default void setTurretPosition(double targetEncPos) {}

    public default double getTurretDeg() {
        return 0;
    }

    public default void setTurretCurrentPosition(double currentEncPos) {}
}
