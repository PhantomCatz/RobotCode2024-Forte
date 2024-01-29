package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorIOInputs {
        public double dummyVariable; //TBD example variable
        public double elevatorVoltage;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void exampleAccessMethod(double test) {}

    public default void setElevatorPosition(double newPositionElevator) {}

    public default void setElevatorPercentOutput(double speed) {}
}
