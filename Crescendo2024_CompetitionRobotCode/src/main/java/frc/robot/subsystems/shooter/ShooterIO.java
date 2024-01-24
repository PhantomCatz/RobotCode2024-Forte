package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double dummyVariable;
        public double velocityBtmLT;
        public double velocityBtmRT;
        public double velocityTopRT;
        public double velocityTopLT;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void shootWithVelocity() {}

    public default void setShooterDisabled() {}
}
