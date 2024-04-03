package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    
    @AutoLog
    public class ClimbIOInputs {
        // public double climbSpoolRevLT;
        // public double climbPositionErrorLT;
        // public double climbSpoolRevRT;
        // public double climbPositionErrorRT;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}


    public default void setClimbPositionLT(double climbPositionRev) {}

    public default void setClimbMtrPercentOutputLT(double speed) {} 

    public default void setClimbSelectedSensorPositionLT(double setNewReadPosition) {}


    public default void setClimbPositionRT(double climbPositionRev) {}

    public default void setClimbMtrPercentOutputRT(double speed) {} 

    public default void setClimbSelectedSensorPositionRT(double setNewReadPosition) {}
}
