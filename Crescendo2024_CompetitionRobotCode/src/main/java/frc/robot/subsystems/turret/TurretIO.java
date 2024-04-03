package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public class TurretIOInputs {
        public double turretMtrPercentOutput;
        public double turretEncValue;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void turretSetPwr(double outputPwr) {}

    public default void turretSetEncoderPos(double position){}
    
}
