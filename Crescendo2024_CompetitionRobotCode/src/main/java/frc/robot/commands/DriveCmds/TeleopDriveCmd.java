package frc.robot.commands.DriveCmds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class TeleopDriveCmd extends Command {

  private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();
  Supplier<Double> supplierLeftJoyX;
  Supplier<Double> supplierLeftJoyY;
  Supplier<Double> supplierRightJoyX;
  Supplier<Double> supplierPwrMode;
  Supplier<Boolean> isFieldOrientedDisabled;

  /** Creates a new TeleopDriveCmd. */
  public TeleopDriveCmd(Supplier<Double> supplierLeftJoyX,
                        Supplier<Double> supplierLeftJoyY,
                        Supplier<Double> supplierRightJoyX,
                        Supplier<Double> supplierPwrMode,
                        Supplier<Boolean> supplierFieldOriented) {
    this.supplierLeftJoyX = supplierLeftJoyX;
    this.supplierLeftJoyY = supplierLeftJoyY;
    this.supplierRightJoyX = supplierRightJoyX;
    this.supplierPwrMode = supplierPwrMode;
    this.isFieldOrientedDisabled = supplierFieldOriented;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //obtain realtime joystick inputs with supplier methods
    double xSpeed = -supplierLeftJoyY.get();
    double ySpeed = -supplierLeftJoyX.get();
    double turningSpeed = -supplierRightJoyX.get();

    // Apply deadbands to prevent modules from receiving unintentional pwr
    xSpeed =       Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed * DriveConstants.MAX_SPEED: 0.0;
    ySpeed =       Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed * DriveConstants.MAX_SPEED: 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed * DriveConstants.MAX_ANGSPEED_RAD_PER_SEC: 0.0;

    //Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (isFieldOrientedDisabled.get()) {
        // Relative to robot
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    } else {
        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                            xSpeed, ySpeed, turningSpeed, m_driveTrain.getRotation2d()
                                                              );
    }

    //send new chassisspeeds object to the drivetrain
    m_driveTrain.driveRobotWithDescritizeDynamics(chassisSpeeds);

    //logging
    Logger.recordOutput("robot xspeed", xSpeed);
    Logger.recordOutput("robot yspeed", ySpeed);
    Logger.recordOutput("robot turnspeed", turningSpeed);
    Logger.recordOutput("robot orientation", m_driveTrain.getRotation2d().getRadians());
    Logger.recordOutput("chassisspeed x speed mtr sec", chassisSpeeds.vxMetersPerSecond);
    Logger.recordOutput("chassisspeed y speed mtr sec", chassisSpeeds.vyMetersPerSecond);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
