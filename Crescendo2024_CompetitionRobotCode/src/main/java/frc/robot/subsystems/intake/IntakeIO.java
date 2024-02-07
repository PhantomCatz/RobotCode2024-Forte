package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs {
        public double pivotMtrEncPos;
        public double pivotMtrPercentOutput;
        public double rollerVoltage;
        public double rollerPercentOutput;
        public double rollerVelocity;
        public boolean BeamBrkFrontBroken;
        public boolean BeamBrkBackBroken;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void exampleAccessMethod(double test) {}

    public default void deployIntake(boolean enable) {}

    public default void rollerEnable(boolean enable) {}

    public default void setRollerPercentOutputIO(double speed) {}

    public default void setRollerVelocity(double velocity) {}

    public default void rollerIn() {}

    public default void rollerOut() {}

    public default void rollerDisable() {}

    public default void setPivotEncPos(double targetEncPos) {}

    public default void resetPivotEncPos(double defaultEncoderPosition) {}

    public default void setIntakePivotPercentOutput(double percentOutput) {}
}
