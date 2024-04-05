package frc.robot.commands.DriveCmds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class TeleopDriveCmd extends Command {
  //subystem declaration collection
  private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();

  private Supplier<Double> m_supplierLeftJoyX;
  private Supplier<Double> m_supplierLeftJoyY;
  private Supplier<Double> m_supplierRightJoyX;
  private Supplier<Boolean> m_isFieldOrientedDisabled;

  //drive variables
  private double xSpeed;
  private double ySpeed;
  private double turningSpeed;

  private ChassisSpeeds chassisSpeeds;


  public TeleopDriveCmd(Supplier<Double> supplierLeftJoyX,
                        Supplier<Double> supplierLeftJoyY,
                        Supplier<Double> supplierRightJoyX,
                        Supplier<Boolean> supplierFieldOriented) {
    this.m_supplierLeftJoyX        = supplierLeftJoyX;
    this.m_supplierLeftJoyY        = supplierLeftJoyY;
    this.m_supplierRightJoyX       = supplierRightJoyX;
    this.m_isFieldOrientedDisabled = supplierFieldOriented;

    addRequirements(m_driveTrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //obtain realtime joystick inputs with supplier methods
    xSpeed =       -m_supplierLeftJoyY.get() * Robot.flipDirection;
    ySpeed =       -m_supplierLeftJoyX.get() * Robot.flipDirection; 
    turningSpeed =  m_supplierRightJoyX.get() * Robot.flipDirection;

    // Apply deadbands to prevent modules from receiving unintentional pwr
    xSpeed =       Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed * DriveConstants.MAX_SPEED: 0.0;
    ySpeed =       Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed * DriveConstants.MAX_SPEED: 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed * DriveConstants.MAX_ANGSPEED_RAD_PER_SEC: 0.0;

    //Construct desired chassis speeds
    if (m_isFieldOrientedDisabled.get()) {
        // Relative to robot
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    } else {
        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                            xSpeed, ySpeed, turningSpeed, m_driveTrain.getRotation2d()
                                                              );
    }

    //send new chassisspeeds object to the drivetrain
    m_driveTrain.driveRobotWithDiscretizeKinematics(chassisSpeeds);

  

  }

  /*
  * For Debugging Purposes 
  * Keep them commmented ALWAYS if you are not using it 
  */
  public void debugLogsDrive(){
    //DEBUG
      // Logger.recordOutput("robot xspeed", xSpeed);
      // Logger.recordOutput("robot yspeed", ySpeed);
      // Logger.recordOutput("robot turnspeed", turningSpeed);
      // Logger.recordOutput("robot orientation", m_driveTrain.getRotation2d().getRadians());
      // Logger.recordOutput("chassisspeed x speed mtr sec", chassisSpeeds.vxMetersPerSecond);
      // Logger.recordOutput("chassisspeed y speed mtr sec", chassisSpeeds.vyMetersPerSecond);
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

