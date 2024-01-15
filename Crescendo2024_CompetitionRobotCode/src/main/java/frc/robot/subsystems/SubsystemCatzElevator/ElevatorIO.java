package frc.robot.subsystems.SubsystemCatzElevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorIOInputs {
        public double dummyVariable;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void exampleAccessMethod(double test) {}
}
