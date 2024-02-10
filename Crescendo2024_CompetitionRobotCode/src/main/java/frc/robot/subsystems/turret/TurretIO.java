package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public class TurretIOInputs {
        public double turretMtrPercentOutput;
        public double turretMtrSupplyCurrent;
        public double turretMtrStatorCurrent;
        public double turretMtrOutputCurrent;
        public double turretEncValue;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void exampleAccessMethod(double test) {}

    public default void deployIntake(boolean enable) {}

    public default void rollerEnable(boolean enable) {}

    public default void setRollerPercentOutputIO(double speed) {}

    public default void setRollerVelocity(double velocity) {}

    public default void rollerIn() {}

    public default void rollerOut() {}

    public default void rollerDisable() {}

    public default void turretRotate(double powerOutput) {}

    public default void turretSet(double outputPwr) {}//targetEncPos) {}

    public default void resetPivotEncPos(double defaultEncoderPosition) {}

    public default void setIntakePivotPercentOutput(double percentOutput) {}

}
