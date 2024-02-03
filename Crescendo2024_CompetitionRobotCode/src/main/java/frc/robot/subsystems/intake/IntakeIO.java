package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs {
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void deployIntake(boolean enable) {}

    public default void rollerEnable(boolean enable) {}

    public default void setPivotEncPos(double targetencoderPostion) {}

    public default void setRollerPercentOutput(double speed) {}

    public default void setRollerVelocity(double velocity) {}

    public default void resetPivotEncPos(double defaultEncoderPosition) {}
}
