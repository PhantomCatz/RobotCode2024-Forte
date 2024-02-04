package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double dummyVariable;
        public double velocityBtmRT;
        public double velocityTopRT;
        public double shooterTopPercentOutput;
        public double shooterBtmPercentOutput;
        public double shooterTopSupplyCurrent;
        public double shooterBtmSupplyCurrent;
        public double shooterTopStatorCurrent;
        public double shooterBtmStatorCurrent;
        public double shooterTopTorqueCurrent;
        public double shooterBtmTorqueCurrent;
        public double feederMotorPercentOutput; 
        public double feederMotorVelocity;
        public double feederMotor2PercentOutput;
        public double feederMotor2Velocity;
        public double turretDeg;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void shootWithVelocity() {}

    public default void setShooterDisabled() {}

    public default void shootFeederReverse() {}

    public default void setFeederDisabled() {}

    public default void setTurretPosition(double targetEncPos) {}

    public default double getTurretDeg() {
        return 0;
    }

    public default void setTurretCurrentPosition(double currentEncPos) {}
}
