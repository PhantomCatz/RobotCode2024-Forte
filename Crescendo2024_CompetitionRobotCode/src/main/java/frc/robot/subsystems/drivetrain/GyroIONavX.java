// package frc.robot.subsystems.drivetrain;

// import com.kauailabs.navx.frc.AHRS;

// import frc.robot.CatzConstants.DriveConstants;
// public class GyroIONavX implements GyroIO 
// {
//     private final AHRS navX;

//     public GyroIONavX() {
//         navX = new AHRS();
//     }

//     @Override
//     public void updateInputs(GyroIOInputs inputs) {
//       inputs.gyroAngle = navX.getAngle();
//       inputs.gyroYaw = navX.getYaw();
//       inputs.gyroRoll = navX.getRoll();
//       inputs.gyroConnected = navX.isConnected();
//     }

//     @Override
//     public void resetNavXIO(){
//         navX.reset();
//         navX.setAngleAdjustment(DriveConstants.defaultRot.getDegrees());
//     }

//     @Override
//     public void setAngleAdjustmentIO(double gyroAdjustment) {
//         navX.setAngleAdjustment(gyroAdjustment);
//     }

//     @Override
//     public double getAngleAdjustmentIO(){
//         return navX.getAngleAdjustment();
//     }
// }

