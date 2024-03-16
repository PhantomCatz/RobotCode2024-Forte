package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.CatzConstants.DriveConstants;
public class GyroIONavX implements GyroIO 
{
    private final AHRS navX;

    public GyroIONavX() {
        navX = new AHRS(Port.kMXP, (byte) 200);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.gyroAngle = navX.getAngle();
      inputs.gyroYaw = navX.getYaw()+180;
      inputs.gyroRoll = navX.getRoll();
      inputs.gyroConnected = navX.isConnected();
    }

    @Override
    public void resetNavXIO(){
        navX.reset();
        navX.setAngleAdjustment(DriveConstants.defaultRot.getDegrees());
    }

    @Override
    public void setAngleAdjustmentIO(double gyroAdjustment) {
        navX.setAngleAdjustment(gyroAdjustment);
    }

    @Override
    public double getAngleAdjustmentIO(){
        return navX.getAngleAdjustment();
    }
}

