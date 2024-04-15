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


  //subsystem declaration
  private SubsystemCatzElevator   elevator   = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake     intake     = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter    shooter    = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret     turret     = SubsystemCatzTurret.getInstance();
  private SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();

  private static final double SHOOTER_VEL_RT_SCALAR = 1.85;

  private boolean isInOffensiveMode = false;
  private double hoardVelLT;
  private double hoardVelRT;

  //------------------------------------------------------------------------------------------------
  //
  // Interpolation tables
  //
  //------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------
  //  Shooter EL angle look up table key: 
  //    Param 1: Distance in meters from center of robot to shot note location
  //    Param 2: RPS shooter motor shaft units for lower rps side (left)
  //------------------------------------------------------------------------------------------------
  private static final InterpolatingDoubleTreeMap shooterVelPivotTable = new InterpolatingDoubleTreeMap();

  static {
    shooterVelPivotTable.put(1.28, 35.0); 

    shooterVelPivotTable.put(1.2, 30.0);

    shooterVelPivotTable.put(0.972, 25.0);
  }

  private Translation2d m_targetXY;


  //constructor for hoard
  public HoardShotCmd(boolean isInOffensiveMode) {
    addRequirements(turret, shooter, intake, elevator);  
    this.isInOffensiveMode = isInOffensiveMode; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.updateAutoTargetPositionIntake(CatzMechanismConstants.AUTO_AIM_PRESET.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(CatzMechanismConstants.AUTO_AIM_PRESET.getElevatorTargetRev());


    //alliance color + hoard Mode target defining
    if(CatzAutonomous.getInstance().getAllianceColor() == CatzConstants.AllianceColor.Blue) { 

      //in blue alliance hoarding mode
      if(isInOffensiveMode) {

        //translation of the blue alliance offensive hoarding location
        m_targetXY = new Translation2d(0.0, FieldConstants.HOARD_LOCATION_Y);
      } else {
              
        //translation of the blue alliance Hoarding Defense location
        m_targetXY = new Translation2d(6.64, 6.96);
      }

    } else {

      //is in red alliance Hoarding
      if(isInOffensiveMode) {
        //translation of the Red alliance Hoarding
        m_targetXY = new Translation2d(0.0 + CatzConstants.FieldConstants.FIELD_LENGTH_MTRS , FieldConstants.HOARD_LOCATION_Y);  
      } else {
        //translation of the Red alliance Hoarding
        m_targetXY = new Translation2d(9.18 , 6.96); 
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newDist = m_targetXY.getDistance(drivetrain.getPose().getTranslation());
    
    //scale shooter velocities based off the lower shooter velocity
    hoardVelRT = shooterVelPivotTable.get(newDist) * SHOOTER_VEL_RT_SCALAR; //max scale
    hoardVelLT = shooterVelPivotTable.get(newDist);                         //min scale


    //set shooter velocities without state machine
    shooter.setFlyWheelVelocities(hoardVelLT, hoardVelRT);

    //auto aim turret
    turret.aimAtGoal(m_targetXY, false);

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
