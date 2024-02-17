package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.logging.Logger;

import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.turret.SubsystemCatzTurret;


public class AutoAlignCmd extends InstantCommand {
  private final SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();
  private final SubsystemCatzTurret     turret     = SubsystemCatzTurret.getInstance();
  private DriverStation.Alliance alliance = null;

  private static final double SPEAKER_COORD_MTRS_Y = Units.inchesToMeters(219.277);
  private static final double FIELD_LENGTH_MTRS = Units.inchesToMeters(651.223);
  private double hozDeg;
  private double driveTrainOffset;


  public AutoAlignCmd() {
    addRequirements(turret);
    turret.setTurretTargetDegree(getHozDeg());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // if(DriverStation.getAlliance().isPresent()) {
    //   this.alliance = DriverStation.getAlliance().get();
    // }

    // if(alliance == DriverStation.Alliance.Red) {
    //   hozDeg = Math.toDegrees(Math.atan2(SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(),
    //                                      FIELD_LENGTH_MTRS    - drivetrain.getPose().getX()));
    // } else {
    //   hozDeg = Math.toDegrees(Math.atan2(SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(),
    //                                     -drivetrain.getPose().getX()));
    // }
    // driveTrainOffset = drivetrain.getPose().getRotation().getDegrees();

    // turret.setTurretTargetDegree(-hozDeg);

    // System.out.println(hozDeg);
  }

  @Override 
  public void execute() {

    if(DriverStation.getAlliance().isPresent()) {
      this.alliance = DriverStation.getAlliance().get();
    }

    if(alliance == DriverStation.Alliance.Red) {
      hozDeg = Math.toDegrees(Math.atan2(SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(),
                                         FIELD_LENGTH_MTRS    - drivetrain.getPose().getX()));
    } else {
      hozDeg = Math.toDegrees(Math.atan2(SPEAKER_COORD_MTRS_Y - drivetrain.getPose().getY(),
                                        -drivetrain.getPose().getX()));
    }
    driveTrainOffset = drivetrain.getPose().getRotation().getDegrees();

    turret.setTurretTargetDegree(-hozDeg * 1.5);

    System.out.println(hozDeg);
  }
  public double getHozDeg() {
    return hozDeg;
  }
}
