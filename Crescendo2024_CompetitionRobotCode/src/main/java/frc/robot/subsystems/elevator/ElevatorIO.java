package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public double elevatorPosRev;
        public double elevatorPositionError;
        
        public boolean bottomSwitchTripped;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorPosition(double newRevPosition, double elevatorFF, boolean limitSwtichPressed) {}

    public default void setElevatorPercentOutput(double speed) {}

    public default void setSelectedSensorPosition(double pos) {}

    public default void setElevatorVoltage(double volts) {}

}