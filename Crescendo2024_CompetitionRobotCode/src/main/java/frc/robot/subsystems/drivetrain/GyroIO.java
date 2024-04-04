package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO 
{
    @AutoLog
    public static class GyroIOInputs {
    public double gyroAngle;
    public double gyroYaw;
    public double gyroRoll;
    public double gyroPitch;
    public boolean gyroConnected;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void resetNavXIO(double angle) {}

  public default void setAngleAdjustmentIO(double gyroYaw) {}

  public default double getAngleAdjustmentIO() {return 0.0;}

}
