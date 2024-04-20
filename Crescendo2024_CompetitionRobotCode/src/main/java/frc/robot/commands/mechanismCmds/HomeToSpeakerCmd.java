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
import frc.robot.subsystems.LEDs.SubsystemCatzLED;
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
  private SubsystemCatzLED        lead       = SubsystemCatzLED.getInstance();

  //------------------------------------------------------------------------------------------------
  //
  // Interpolation tables
  //
  //------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------
  //  Shooter EL angle look up table key: 
  //    Param 1: Distance in meters from back wall to Center of the robot
  //    Param 2: pivot position % of max elevation units
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

  private Translation2d m_targetXY;
  private double newDist;
  private double servoPos;


  private final double AUTON_TIMEOUT_SEC = 4.0;
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

  @Override
  public void initialize() {

    intake.updateAutoTargetPositionIntake(CatzMechanismConstants.AUTO_AIM_PRESET.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(CatzMechanismConstants.AUTO_AIM_PRESET.getElevatorTargetRev());

    if(DriverStation.isTeleop()) {
      shooter.startShooterFlywheel(); 
    }

    if(CatzAutonomous.getInstance().getAllianceColor() == CatzConstants.AllianceColor.Blue) {  
      
      //translation of the blue alliance speaker
      m_targetXY = new Translation2d(0.0, FieldConstants.SPEAKER_COORD_MTRS_Y);

    } else {
      //translation of the Red alliance speaker
      m_targetXY = new Translation2d(0.0 + CatzConstants.FieldConstants.FIELD_LENGTH_MTRS , FieldConstants.SPEAKER_COORD_MTRS_Y);     
    }
    turret.setTurretInPose(false);

    Logger.recordOutput("Speaker", m_targetXY);

    timer.reset();
    timer.start();

  }

  
  //------------------------------------------------------------------------------------------------
  //
  //  execute()
  //
  //------------------------------------------------------------------------------------------------
  @Override 
  public void execute() {
  
       servoPos = shooterPivotTable.get(newDist);

         //use pose to pose linear interpolation table for servo
       newDist = m_targetXY.getDistance(drivetrain.getPose().getTranslation());
       servoPos = shooterPivotTable.get(newDist);

    

       shooter.updateShooterServo(servoPos);
      
      turret.aimAtGoal(m_targetXY, false); //change back to false if auto aim doesn't work
      
    if(DriverStation.isAutonomous()){

      if((turret.getTurretInPos() && 
          shooter.isAutonShooterRamped() //&& 
          //timer.hasElapsed(1.0)
          )) 
        {
        //mechanisms are in position
          shooter.setShooterState(ShooterState.SHOOTING);
          System.out.println("mech in pos");

      } else {

        if(timer.hasElapsed(AUTON_TIMEOUT_SEC)) {
            // timeout has been reached
            shooter.setShooterState(ShooterState.SHOOTING);

            System.out.println("timeout");
          

        }  
      }
      }
    }

  @Override
  public boolean isFinished(){
    return shooter.getShooterNoteState() == ShooterNoteState.NOTE_HAS_BEEN_SHOT;
  }


}
