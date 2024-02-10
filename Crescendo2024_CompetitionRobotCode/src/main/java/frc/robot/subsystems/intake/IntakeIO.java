package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs {
        public double pivotMtrEncRev;
        public double pivotMtrPercentOutput;
        public double rollerVoltage;
        public double rollerPercentOutput;
        public double rollerVelocity;
        public boolean BeamBrkFrontBroken;
        public boolean BeamBrkBackBroken;
        public double closedLoopPivotMtr;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void rollerEnable(boolean enable) {}

    public default void setRollerPercentOutput(double speed) {}

    public default void rollerIn() {}

    public default void rollerOut() {}

    public default void rollerDisable() {}

    public default void resetPivotEncPos(double defaultEncoderPosition) {}

    public default void setIntakePivotPercentOutput(double percentOutput) {}
}
