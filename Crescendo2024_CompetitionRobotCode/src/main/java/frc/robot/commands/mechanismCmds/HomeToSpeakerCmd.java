package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.littletonrobotics.junction.Logger;

import java.awt.geom.Point2D;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.FieldConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.FieldRelativeAccel;
import frc.robot.Utils.FieldRelativeSpeed;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.elevator.SubsystemCatzElevator.ElevatorControlState;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeControlState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterNoteState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.vision.SubsystemCatzVision;


public class HomeToSpeakerCmd extends Command {

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
  private static final InterpolatingDoubleTreeMap newShooterPivotTable = new InterpolatingDoubleTreeMap();

  static {
    newShooterPivotTable.put(1.478, 1.0);
    newShooterPivotTable.put(1.875, 0.82);

    newShooterPivotTable.put(1.875, 9.5);
    
    //UNTESTED VALUES
    newShooterPivotTable.put(2.37, 0.625);
    //shooterPivotTable.put(2.37, 0.350);     //93.30709
    //shooterPivotTable.put(2.37, 0.300);

    newShooterPivotTable.put(2.87, 0.590); 
    //shooterPivotTable.put(2.87, 0.300);     //112.9921
    //shooterPivotTable.put(2.87, 0.280); 

    newShooterPivotTable.put(3.37, 0.525);
    //shooterPivotTable.put(3.37, 0.250);     //132.6772
    //shooterPivotTable.put(3.37, 0.200);

    newShooterPivotTable.put(3.87, 0.412);
    //shooterPivotTable.put(3.87, 0.125);     //152.3622  
    //shooterPivotTable.put(3.87, 0.100);

    newShooterPivotTable.put(4.87, 0.370);
    //shooterPivotTable.put(4.87, 0.100);     //191.7323
    //shooterPivotTable.put(4.87, 0.050);

    newShooterPivotTable.put(5.87, 0.300);     //231.1024
  }


  public static final double k_ACCEL_COMP_FACTOR = 0.100; // in units of seconds    TBD Where is this used?

  private Translation2d m_targetXY;

  //------------------------------------------------------------------------------------------------
  //
  //  
  //
  //------------------------------------------------------------------------------------------------
  private Supplier<Boolean> m_supplierButtonB;
  

  //for telop

  //for autonomous
  public HomeToSpeakerCmd() {                     
    addRequirements(turret, shooter, intake, elevator);
  }

  
  //------------------------------------------------------------------------------------------------
  //
  //  initialize()
  //
  // Called when the command is initially scheduled.
  //------------------------------------------------------------------------------------------------
  @Override
  public void initialize() {

    intake.updateAutoTargetPositionIntake(CatzMechanismConstants.AUTO_AIM_PRESET.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(CatzMechanismConstants.AUTO_AIM_PRESET.getElevatorTargetRev());
    
    if(CatzAutonomous.getInstance().getAllianceColor() == CatzConstants.AllianceColor.Blue) {    //TBD - we should do this once on startup vs every cmd call //TTTchanging to red 
      
      //translation of the blue alliance speaker
      m_targetXY = new Translation2d(0.0, FieldConstants.SPEAKER_COORD_MTRS_Y);

    } else {
      //translation of the Red alliance speaker
      m_targetXY = new Translation2d(0.0 + CatzConstants.FieldConstants.FIELD_LENGTH_MTRS , FieldConstants.SPEAKER_COORD_MTRS_Y);      //TBD - Magic #'s, what about defining Red & Blue constants, using IF to select and have 1 translation2D() call
    }


  }

  
  //------------------------------------------------------------------------------------------------
  //
  //  execute()
  //
  //------------------------------------------------------------------------------------------------
  @Override 
  public void execute() {

    double newDist = m_targetXY.getDistance(drivetrain.getPose().getTranslation());
    double servoPos = newShooterPivotTable.get(newDist);
    turret.aimAtGoal(m_targetXY, false, false);    
    shooter.updateShooterServo(servoPos);

    //in telop this boolean supplier is being evaluated to see if button was pressed

    if(DriverStation.isAutonomous()){
      if(shooter.getShooterServoInPos() && turret.isTurretAtTarget()){ //TBD add the timer code for shooter pivot
        shooter.setShooterState(ShooterState.SHOOTING);
      }
    }

    Logger.recordOutput("ShooterCalcs/NewDist",           newDist);
    Logger.recordOutput("servoCmdPos",                    servoPos);

  }

  @Override
  public boolean isFinished(){
    return shooter.getShooterNoteState() == ShooterNoteState.NOTE_HAS_BEEN_SHOT;
  }


}
