package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.FieldConstants;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterNoteState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;


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
  private static final InterpolatingDoubleTreeMap shooterPivotTable = new InterpolatingDoubleTreeMap();

  static {
    shooterPivotTable.put(1.478, 1.0);

    shooterPivotTable.put(1.875, 0.885);
    // newShooterPivotTable.put(1.875, 0.82);
    // newShooterPivotTable.put(1.875, 0.95);

    shooterPivotTable.put(2.875, 0.485);
    // newShooterPivotTable.put(2.875, 0.42);
    // newShooterPivotTable.put(2.875, 0.55);
    
    shooterPivotTable.put(3.875, 0.26);
    // newShooterPivotTable.put(3.875, 0.21);
    // newShooterPivotTable.put(3.875, 0.31);
    
    shooterPivotTable.put(4.875, 0.095);
    // newShooterPivotTable.put(4.875, 0.09);
    // newShooterPivotTable.put(4.875, 0.1);

    shooterPivotTable.put(5.875, 0.02);
    // newShooterPivotTable.put(5.875, 0.0);
    // newShooterPivotTable.put(5.875, 0.04);

    shooterPivotTable.put(6.813, 0.0);
  }


  public static final double k_ACCEL_COMP_FACTOR = 0.100; // in units of seconds    TBD Where is this used?

  private Translation2d m_targetXY;

  private static final double AUTON_TIMEOUT_SEC = 1.0;

  //------------------------------------------------------------------------------------------------
  //
  //  
  //
  //------------------------------------------------------------------------------------------------
  private Timer timer = new Timer();

  public HomeToSpeakerCmd() {                     
    addRequirements(turret, shooter, intake, elevator);
  }

  
  //------------------------------------------------------------------------------------------------
  //
  //  initialize()
  //
  // Called when the command is initially scheduled.
  //------------------------------------------------------------------------------------------------

  private double prevTime = 0.0;
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    intake.updateAutoTargetPositionIntake(CatzMechanismConstants.AUTO_AIM_PRESET.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(CatzMechanismConstants.AUTO_AIM_PRESET.getElevatorTargetRev());
    
    if(CatzAutonomous.getInstance().getAllianceColor() == CatzConstants.AllianceColor.Blue) {    //TBD - we should do this once on startup vs every cmd call //TTTchanging to red 
      
      //translation of the blue alliance speaker
      m_targetXY = new Translation2d(0.0, FieldConstants.SPEAKER_COORD_MTRS_Y);

    } else {
      //translation of the Red alliance speaker
      m_targetXY = new Translation2d(0.0 + CatzConstants.FieldConstants.FIELD_LENGTH_MTRS , FieldConstants.SPEAKER_COORD_MTRS_Y);      //TBD - Magic #'s, what about defining Red & Blue constants, using IF to select and have 1 translation2D() call
    }
    turret.setTurretInPose(false);

  }

  
  //------------------------------------------------------------------------------------------------
  //
  //  execute()
  //
  //------------------------------------------------------------------------------------------------
  @Override 
  public void execute() {

    if(CatzAutonomous.test == false){

      prevTime = timer.get();
      double newDist = m_targetXY.getDistance(drivetrain.getPose().getTranslation());
      System.out.println(timer.get()-prevTime + " newDist");
  
      prevTime = timer.get();
      double servoPos = shooterPivotTable.get(newDist);
      System.out.println(timer.get()-prevTime + " servoPos");
  
      prevTime = timer.get();
      turret.aimAtGoal(m_targetXY, false, false);
      System.out.println(timer.get()-prevTime + " aimAtGoal");
  
      prevTime = timer.get();
      shooter.updateShooterServo(servoPos);
      System.out.println(timer.get()-prevTime + " updateServo");
  
      //in telop this boolean supplier is being evaluated to see if button was pressed
  
      // System.out.println("turret:"+turret.isTurretAtTarget());
      // System.out.println("shooter:"+shooter.isAutonShooterRamped());
      // System.out.println("timer:"+timer.hasElapsed(AUTON_TIMEOUT_SEC));
  
      prevTime = timer.get();
      if(DriverStation.isAutonomous()){
        System.out.println(timer.get()-prevTime + " isAutonomous");
  
        prevTime = timer.get();
        if((/*shooter.getShooterServoInPos() && */ turret.getTurretInPos() && shooter.isAutonShooterRamped() && timer.hasElapsed(AUTON_TIMEOUT_SEC))) { //TBD add the timer code for shooter pivot
          System.out.println(timer.get()-prevTime + " checkForShoot");
  
          prevTime = timer.get();
          shooter.setShooterState(ShooterState.SHOOTING);
          System.out.println(timer.get()-prevTime + " SHOOT!");
        }
      }
      CatzAutonomous.test = true;
    }else{
      double newDist = m_targetXY.getDistance(drivetrain.getPose().getTranslation());
  
      double servoPos = shooterPivotTable.get(newDist);
  
      turret.aimAtGoal(m_targetXY, false, false);
  
      shooter.updateShooterServo(servoPos);
  
      //in telop this boolean supplier is being evaluated to see if button was pressed
  
      // System.out.println("turret:"+turret.isTurretAtTarget());
      // System.out.println("shooter:"+shooter.isAutonShooterRamped());
      // System.out.println("timer:"+timer.hasElapsed(AUTON_TIMEOUT_SEC));
  
      if(DriverStation.isAutonomous()){
  
        if((/*shooter.getShooterServoInPos() && */ turret.getTurretInPos() && shooter.isAutonShooterRamped() && timer.hasElapsed(AUTON_TIMEOUT_SEC))) { //TBD add the timer code for shooter pivot
  
          shooter.setShooterState(ShooterState.SHOOTING);
        }
      }
      Logger.recordOutput("ShooterCalcs/NewDist",           newDist);
      Logger.recordOutput("servoCmdPos",                    servoPos);
    }



  }

  @Override
  public boolean isFinished(){
    return shooter.getShooterNoteState() == ShooterNoteState.NOTE_HAS_BEEN_SHOT;
  }


}