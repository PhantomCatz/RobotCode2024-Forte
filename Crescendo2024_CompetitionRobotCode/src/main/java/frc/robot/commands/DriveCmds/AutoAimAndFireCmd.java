// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCmds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Vector;

// import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.FieldConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class AutoAimAndFireCmd extends Command {
  private final SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();
  private final SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();
  private final PIDController controller;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  private static double minVelocity = 0;
  private static double toleranceDegrees = 0;
  private DriverStation.Alliance alliance = null;

  //vector shooting
  private double drivetrainVelocityMPS = 0.0;
  private final double NOTE_EJECT_SPEED_MPS = 9.0;

  private static final double VELOCITY_VECTOR_INDEX = 0;
  private static final double X_COMPONENT_VECOTOR_INDEX = 1;
  private static final double Y_COMPONENT_VECTOR_INDEX = 2;


  public AutoAimAndFireCmd() {
    addRequirements(drivetrain);
    switch (CatzConstants.currentMode) {
      case REAL:
        kP = 0.018;
        kI = 0.0;
        kD = 0.0;
        minVelocity = 0.0;
        toleranceDegrees = 1.0;
        break;
      default: // for SIM
        kP = 0.1;
        kI = 0.0;
        kD = 0.001;
        minVelocity = 0.0;
        toleranceDegrees = 1.5;
        break;
    }

    controller = new PIDController(kP, kI, kD, 0.02);
    controller.setTolerance(toleranceDegrees);
    controller.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();

    if (DriverStation.getAlliance().isPresent()) {
      this.alliance = DriverStation.getAlliance().get();
    }

    //set the target speaker setpoint by geting the arc tangent angle error and converting to degrees
    if (alliance == DriverStation.Alliance.Red) {
      controller.setSetpoint(
          Math.toDegrees(
                  Math.atan2(FieldConstants.SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(),
                             FieldConstants.FIELD_LENGTH_MTRS    - drivetrain.getPose().getX())));
    } else {
      controller.setSetpoint(
          Math.toDegrees(
                  Math.atan2(FieldConstants.SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(), 
                             -drivetrain.getPose().getX())));
    }
    double driveRotaitonOffset = (drivetrain.getPose().getRotation().getDegrees());
    double globalTargetAimAngle =  Math.toDegrees(
                                    Math.atan2(FieldConstants.SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(),
                                              FieldConstants.FIELD_LENGTH_MTRS    - drivetrain.getPose().getX()));

    double targetTurretAngle = globalTargetAimAngle - 
                                      Math.signum(driveRotaitonOffset) * Math.toDegrees(
                                                                                CatzMathUtils.toUnitCircAngle(
                                                                                                Math.toRadians(driveRotaitonOffset)));
    
  }

  @Override
  public void execute() {
    // Update angular speed to send to chassis
    double angularSpeed = controller.calculate(drivetrain.getGyroAngle());

    //less than req speed...then use minmum velocity with the curent anglular sign value
    if (Math.abs(angularSpeed) < minVelocity) {
      angularSpeed = Math.copySign(minVelocity, angularSpeed);
    }
    //send to the drivetrain
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0, angularSpeed);
    drivetrain.driveRobotWithDescritizeDynamics(chassisSpeeds);

    //logging
    Logger.recordOutput("Angular Speed", angularSpeed);
    Logger.recordOutput("AutoAimAndFireActive", true);
    Logger.recordOutput("Turn X Goal", controller.getSetpoint());

    //---------------------------------------------------------
    //vector aiming
    //----------------------------------------------------------
    Pose2d vectorCalculationPose = drivetrain.getPose();
    //vector aiming
    double globalCurrentTurretAngle = vectorCalculationPose.getRotation().getDegrees() + SubsystemCatzTurret.getInstance().getTurretAngle();

    //arrays used to represent vectors
    Double[] drivetrainVelocityVector =  {drivetrainVelocityMPS, vectorCalculationPose.getRotation().getCos(), vectorCalculationPose.getRotation().getSin()};
    Double[] noteDirectionVector =       {NOTE_EJECT_SPEED_MPS, Math.cos(globalCurrentTurretAngle), Math.sin(globalCurrentTurretAngle)};

    //take the driving vector and subtract it from Current Turret driection vector to get the new target vector
    Double[] finalNoteVector    =  {((drivetrainVelocityVector[0] * drivetrainVelocityVector[1]) - (noteDirectionVector[0] * noteDirectionVector[1])),
                                    ((drivetrainVelocityVector[0] * drivetrainVelocityVector[2]) - (noteDirectionVector[0] * noteDirectionVector[2]))};

    double calculatedTurretAngle = Math.atan(finalNoteVector[1]/finalNoteVector[0]);

    
    turret.cmdTurretDegree(calculatedTurretAngle);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("AutoAimAndFireActive", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
