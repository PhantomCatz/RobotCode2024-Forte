package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public interface ModuleIO {
 @AutoLog
 static class ModuleIOInputs {
    public double driveMtrVelocity = 0.0;
    public double driveMtrSensorPosition = 0.0;
    public double magEncoderValue = 0.0;
    public double driveAppliedVolts = 0.0;
    public double steerAppliedVolts = 0.0;
    public double driveVelocityError = 0.0;
 }

 /** Updates the set of loggable inputs. */
 public default void updateInputs(ModuleIOInputs inputs) {}

 public default void setDrivePwrPercentIO(double drivePwrPercent) {}

 public default void setDriveVelocityIO(double velocity) {}

 public default void setSteerPwrIO(double SteerPwr) {}

 public default void setSteerCoastModeIO() {}

 public default void setSteerBrakeModeIO() {}

 public default void setDrvSensorPositionIO(double sensorpos) {}

 public default void reverseDriveIO(boolean enable) {}

 public default void setDriveSimPwrIO(double volts) {}

 public default void setSteerSimPwrIO(double volts) {}

 public default void resetMagEncoderIO() {}

}
