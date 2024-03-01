// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.DriveCmds;

// import org.littletonrobotics.junction.Logger;

// import com.google.flatbuffers.Constants;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.CatzConstants;
// import frc.robot.CatzConstants.DriveConstants;
// import frc.robot.CatzConstants.FieldConstants;
// import frc.robot.Utils.CatzMathUtils;
// import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

// public class AutoAimAndFireCmd extends Command {
//   private final SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();
//   private final PIDController controller;
//   private static double kP = 0;
//   private static double kI = 0;
//   private static double kD = 0;
//   private static double minVelocity = 0;
//   private static double toleranceDegrees = 0;
//   private DriverStation.Alliance alliance = null;
//   /** Creates a new AutoAimAndFireCmd. */
//   public AutoAimAndFireCmd() {
//     addRequirements(drivetrain);
//     switch (CatzConstants.currentMode) {
//       case REAL:
//         kP = 0.018;
//         kI = 0.0;
//         kD = 0.0;
//         minVelocity = 0.0;
//         toleranceDegrees = 1.0;
//         break;
//       default: // for SIM
//         kP = 0.1;
//         kI = 0.0;
//         kD = 0.001;
//         minVelocity = 0.0;
//         toleranceDegrees = 1.5;
//         break;
//     }

//     controller = new PIDController(kP, kI, kD, 0.02);
//     controller.setTolerance(toleranceDegrees);
//     controller.enableContinuousInput(-180, 180);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     controller.reset();

//     if (DriverStation.getAlliance().isPresent()) {
//       this.alliance = DriverStation.getAlliance().get();
//     }

//     //set the target speaker setpoint by geting the arc tangent angle error and converting to degrees
//     if (alliance == DriverStation.Alliance.Red) {
//       controller.setSetpoint(
//           Math.toDegrees(
//                   Math.atan2(FieldConstants.SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(),
//                              FieldConstants.FIELD_LENGTH_MTRS    - drivetrain.getPose().getX())));
//     } else {
//       controller.setSetpoint(
//           Math.toDegrees(
//                   Math.atan2(FieldConstants.SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(), 
//                              -drivetrain.getPose().getX())));
//     }
//     double driveRotaitonOffset = (drivetrain.getPose().getRotation().getDegrees());
//     double globalTargetAimAngle =  Math.toDegrees(
//                                     Math.atan2(FieldConstants.SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(),
//                                               FieldConstants.FIELD_LENGTH_MTRS    - drivetrain.getPose().getX()));

//     double targetTurretAngle = globalTargetAimAngle - 
//                                       Math.signum(driveRotaitonOffset) * Math.toDegrees(
//                                                                                 CatzMathUtils.toUnitCircAngle(
//                                                                                                 Math.toRadians(driveRotaitonOffset)));
//   }

//   @Override
//   public void execute() {
//     // Update angular speed to send to chassis
//     double angularSpeed = controller.calculate(drivetrain.getGyroAngle());

//     //less than req speed...then use minmum velocity with the curent anglular sign value
//     if (Math.abs(angularSpeed) < minVelocity) {
//       angularSpeed = Math.copySign(minVelocity, angularSpeed);
//     }
//     //send to the drivetrain
//     ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0, angularSpeed);
//     drivetrain.driveRobotWithDescritizeDynamics(chassisSpeeds);

//     //logging
//     Logger.recordOutput("Angular Speed", angularSpeed);
//     Logger.recordOutput("AutoAimAndFireActive", true);
//     Logger.recordOutput("Turn X Goal", controller.getSetpoint());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     Logger.recordOutput("AutoAimAndFireActive", false);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return controller.atSetpoint();
//   }
// }
