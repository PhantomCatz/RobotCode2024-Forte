package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.CatzColorConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.CatzConstants.CatzMechanismConstants.robotMode;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.mechanismCmds.MoveToPresetHandoffCmd;
import frc.robot.commands.mechanismCmds.MoveToPreset;
import frc.robot.commands.mechanismCmds.ScoreAmpCmd;
import frc.robot.commands.mechanismCmds.ScoreTrapCmd;
import frc.robot.commands.mechanismCmds.ClimbCmd;
import frc.robot.commands.mechanismCmds.StowPoseCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.MoveToAmpTransition;
import frc.robot.commands.mechanismCmds.AimAndOrFireAtSpeakerCmd;
import frc.robot.subsystems.CatzStateMachine;
import frc.robot.subsystems.CatzStateMachine.NoteDestination;
import frc.robot.subsystems.CatzStateMachine.NoteSource;
import frc.robot.subsystems.LEDs.SubsystemCatzLED;
import frc.robot.subsystems.climb.SubsystemCatzClimb;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.vision.SubsystemCatzVision;

 public class RobotContainer {
    
  //subsystems
  private SubsystemCatzDrivetrain driveTrain; 
  private SubsystemCatzVision     vision;
  private SubsystemCatzIntake     intake;
  private SubsystemCatzShooter    shooter;
  private SubsystemCatzClimb      climb;
  private SubsystemCatzElevator   elevator;
  private SubsystemCatzTurret     turret;
  private SubsystemCatzLED        led;

  private CatzStateMachine stateMachine;

  private CatzAutonomous auton = new CatzAutonomous();

  //xbox controller
  private CommandXboxController xboxDrv;
  private CommandXboxController xboxAux;

  public RobotContainer() {
    //instantiate subsystems
    elevator   = SubsystemCatzElevator.getInstance();
    driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    vision     = SubsystemCatzVision.getInstance();
    intake     = SubsystemCatzIntake.getInstance();
    turret     = SubsystemCatzTurret.getInstance();
    shooter    = SubsystemCatzShooter.getInstance();
    climb      = SubsystemCatzClimb.getInstance();

    stateMachine = CatzStateMachine.getInstance();
    

    xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
    xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);

    // Configure the trigger bindings and default cmds
    defaultCommands();
    configureBindings();
  }
  

  private void configureBindings() {    
    
      //RESET GYRO
      xboxDrv.start().onTrue(driveTrain.resetGyro());

    //------------------------------------------------------------------------------------
    // INTAKE COMMANDS
    //------------------------------------------------------------------------------------
      xboxDrv.leftBumper ().onTrue(intake.cmdRollerIn());      // intake rollers in 
      xboxDrv.rightBumper().onTrue(intake.cmdRollerOut());     // intake rollers out
      xboxDrv.b().onTrue(intake.cmdRollerOff());               // intake rollers off
   
     
      xboxDrv.rightStick().onTrue(Commands.parallel(new StowPoseCmd(),               //STOW INTAKE 
                                                    driveTrain.cancelTrajectory())); //TBD LC
    
      
    //------------------------------------------------------------------------------------
    
     xboxAux.back().onTrue(Commands.runOnce(()-> led.signalHumanPlayerAMP())); // SIGNAL HUMAN PLAYER FOR AMP SCORING

      switch (CatzConstants.CatzMechanismConstants.driverCurrentMode) {

          case SPEAKER_MODE:  
              
              xboxDrv.leftStick().onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)); //DEPLOY INTAKE & STOWS & STORES TO SHOOTER

              xboxAux.rightTrigger().onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.FROM_INTAKE));//NOTE IN INTAKE TRANSFER TO SHOOTER

              xboxAux.x().onTrue(shooter.cmdServoPosition(1.0));

              xboxAux.a().onTrue(shooter.rampUpFlyWheels());  //RAMPING UP 

              xboxAux.b().onTrue(shooter.cmdShoot());                  //TO SHOOT (NEED TO RAMP UP FIRST)

              xboxAux.y().onTrue(new AimAndOrFireAtSpeakerCmd());      //TO AUTO AIM TURRET+SERVOS TO SPEAKER 

              xboxAux.rightStick().onTrue(shooter.cmdServoPosition(xboxAux.getRightY())); //MOVE SERVO POSITION MANUAL 

              xboxAux.rightStick().onTrue(turret.rotate(xboxAux.getRightX()));            //MOVE TURRET POSITION MANUAL

          break;
          //------------------------------------------------------------------------------------
          case AMP_MODE:
              xboxDrv.leftStick().onTrue(new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.INTAKE_GROUND)); //DEPLOY INTAKE AND STOWS TO AMP SCORE DOWN POS

              xboxAux.leftTrigger().onTrue(new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER)); //NOTE IN SHOOTER TRANSFERED TO INTAKE

              xboxAux.y().onTrue(new MoveToAmpTransition());                      //MOVE TO AMP TRANSITION POSITION 

              xboxAux.b().onTrue(new ScoreAmpCmd());                              //SCORE AMP (^^ MUST BE IN AMP TRANSITION POS FIRST)

              xboxAux.leftStick().onTrue(new ManualElevatorCmd((()->xboxAux.getRightY()))); //MANUAL MODE FOR ELEVATOR

          break;
          //------------------------------------------------------------------------------------
          case HOARD_MODE: // HOARD MODE IS NOT DONE BECAUSE WE STILL NEED TO WRITE CODE FOR IT 
              xboxAux.y().onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND));  //PICK UP GROUND TO SHOOTER

              xboxAux.rightBumper().onTrue(shooter.rampUpFlyWheels()); //RAMPING UP
                
              xboxAux.b().onTrue(shooter.cmdShoot());                  //TO SHOOT (NEED TO RAMP UP FIRST)

              xboxAux.x().onTrue(new AimAndOrFireAtSpeakerCmd());      //TO AUTO AIM TURRET+SERVOS TO SPEAKER

              xboxAux.a().onTrue(intake.cmdRollerOut());               // INTAKE ROLLERS SHOOT


          break;
          //------------------------------------------------------------------------------------
    
          case CLIMB_MODE:
              xboxAux.povUp().onTrue(new ClimbCmd(()-> xboxDrv.getLeftY(), ()-> xboxDrv.getRightY()));
              xboxAux.y().onTrue(new ScoreTrapCmd());
              xboxAux.b().onTrue(intake.cmdRollerOut());

          break;

          //------------------------------------------------------------------------------------   

          case CLIMB_MAINTENANCE_MODE:
              xboxAux.povUp().onTrue(new ClimbCmd(()-> xboxDrv.getLeftY(), ()-> xboxDrv.getRightY()));
          break;

          //------------------------------------------------------------------------------------

          default:
              CatzConstants.CatzMechanismConstants.driverCurrentMode = robotMode.SPEAKER_MODE;
          break;

      }
    //------------------------------------------------------------------------------------  


    // //------------------------------------------------------------------------------------
    // //  CLIMB COMMANDS
    // //------------------------------------------------------------------------------------
    //   xboxAux.povUp().onTrue(new ClimbCmd(()-> xboxDrv.getLeftY(), ()-> xboxDrv.getRightY()));



    // //------------------------------------------------------------------------------------
    // //  TRAP COMMAND
    // //------------------------------------------------------------------------------------
    //   xboxAux.povUp().and(xboxAux.b()).onTrue(new ScoreTrapCmd());
    

      
    //------------------------------------------------------------------------------------
    //  CHANGING MODES
    //------------------------------------------------------------------------------------
        xboxAux.povUp().and(xboxDrv.povUp()).onTrue(Commands.runOnce(()-> CatzConstants.CatzMechanismConstants.driverCurrentMode = robotMode.CLIMB_MODE)); // CLIMB MODE

        xboxAux.rightBumper().and(xboxAux.leftBumper()).onTrue(Commands.runOnce(()->CatzConstants.CatzMechanismConstants.driverCurrentMode = robotMode.CLIMB_MAINTENANCE_MODE)); //CLIMB MANTAINANCE MODE

        xboxAux.povDown().onTrue(Commands.runOnce(()->CatzConstants.CatzMechanismConstants.driverCurrentMode = robotMode.HOARD_MODE));                     //HOARD MODE
      
        xboxAux.povLeft().onTrue(Commands.runOnce(()-> CatzConstants.CatzMechanismConstants.driverCurrentMode = robotMode.AMP_MODE));                      //AMP MODE
      
        xboxAux.povRight().onTrue(Commands.runOnce(()->CatzConstants.CatzMechanismConstants.driverCurrentMode = robotMode.SPEAKER_MODE));                  //SPEAKER MODE
    //------------------------------------------------------------------------------------
  }

  public void logDpadStates() {
       SmartDashboard.putString("Scoring Mode", CatzConstants.CatzMechanismConstants.driverCurrentMode.toString());

  }


  //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
  private void defaultCommands() {  
    driveTrain.setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
                                                    ()-> xboxDrv.getLeftY(),
                                                    ()-> -xboxDrv.getRightX(),
                                                    ()-> xboxDrv.b().getAsBoolean()));

  }

  public Command getAutonomousCommand() {
    return auton.getCommand();
  }

}
