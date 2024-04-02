package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs {
        public double pivotMtrRev;
        public double pivotMtrPercentOutput;
        public double pivotMtrVelocityRPS;
        public double rollerVoltage;
        public double rollerPercentOutput;
        public double rollerVelocity;
        public boolean AdjustBeamBrkState;
        public boolean LoadBreamBrkState;
        public double closedLoopPivotMtr;
        public double pivotMtrCurrent;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void rollerEnable(boolean enable) {}

    public default void setRollerPercentOutput(double speed) {}

    public default void rollerIn() {}

    public default void rollerOut() {}

    public default void rollerDisable() {}

    public default void setSquishyMode(boolean enable) {} 

    public default void resetPivotEncPos(double defaultEncoderPosition) {}

    public default void setIntakePivotVoltage(double volts) {}

    public default void setIntakePivotPercentOutput(double percentOutput) {}

    public default void setIntakePivotPostionRev(double pivotEncOuput, double ffVolts) {}
}
