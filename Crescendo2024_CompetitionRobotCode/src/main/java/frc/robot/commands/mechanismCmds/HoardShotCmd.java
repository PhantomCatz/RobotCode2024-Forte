// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.FieldConstants;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class HoardShotCmd extends Command {

  private double hoardVelLT;
  private double hoardVelRT;

  private static final double SHOOTER_VEL_DIFFERENCE = 23.0;

  //subsystem declaration
  private SubsystemCatzElevator   elevator   = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake     intake     = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter    shooter    = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret     turret     = SubsystemCatzTurret.getInstance();
  private SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();

  //------------------------------------------------------------------------------------------------
  //
  // Interpolation tables
  //
  //------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------
  //  Shooter EL angle look up table key: 
  //    Param 1: Distance in meters from back wall to Center of the robot
  //    Param 2: pivot position % of max elevation units
  // TBD - how did we determine distance interval?
  // TBD - explain why two distance values
  //------------------------------------------------------------------------------------------------
  private static final InterpolatingDoubleTreeMap shooterVelPivotTable = new InterpolatingDoubleTreeMap();

  static {
    shooterVelPivotTable.put(1.478, 1.0); //TODO need to test values

    shooterVelPivotTable.put(1.875, 0.885);
    // newShooterPivotTable.put(1.875, 0.82);
    // newShooterPivotTable.put(1.875, 0.95);
  }

  private Translation2d m_targetXY;

  /** Creates a new HomeToHoardShotCmd. */
  public HoardShotCmd() {
    addRequirements(turret, shooter, intake, elevator);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.updateAutoTargetPositionIntake(CatzMechanismConstants.AUTO_AIM_PRESET.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(CatzMechanismConstants.AUTO_AIM_PRESET.getElevatorTargetRev());
    turret.updateTargetPositionTurret(CatzMechanismConstants.STOW_PRESET);

    if(CatzAutonomous.getInstance().getAllianceColor() == CatzConstants.AllianceColor.Blue) {    //TBD - we should do this once on startup vs every cmd call //TTTchanging to red 
      
      //translation of the blue alliance Hoarding
      m_targetXY = new Translation2d(0.0, FieldConstants.HOARD_LOCATION_Y);

    } else {
      //translation of the Red alliance Hoarding
      m_targetXY = new Translation2d(0.0 + CatzConstants.FieldConstants.FIELD_LENGTH_MTRS , FieldConstants.HOARD_LOCATION_Y);     
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newDist = m_targetXY.getDistance(drivetrain.getPose().getTranslation());
    
    hoardVelRT = shooterVelPivotTable.get(newDist);
    hoardVelLT = shooterVelPivotTable.get(newDist) - SHOOTER_VEL_DIFFERENCE;

    shooter.setFlyWheelVelocities(hoardVelLT, hoardVelRT);
    turret.aimAtGoal(m_targetXY, false);

  }

  public double setVelocitySpin(double velRT){ //jank calculation for spin
    double value = velRT - (velRT*0.2);
    return value;
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
