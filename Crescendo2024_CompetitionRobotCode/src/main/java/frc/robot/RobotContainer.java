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
import frc.robot.CatzConstants.CatzMechanismConstants.RobotMode;
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

    //------------------------------------------------------------------------------------
    // SPEAKER MODE
    //------------------------------------------------------------------------------------
        xboxDrv.leftStick().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.SPEAKER_MODE).onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)); //DEPLOY INTAKE & STOWS & STORES TO SHOOTER

        xboxAux.rightTrigger().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.SPEAKER_MODE).onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.FROM_INTAKE));//NOTE IN INTAKE TRANSFER TO SHOOTER

        xboxAux.x().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.SPEAKER_MODE).onTrue(new MoveToPreset(CatzConstants.CatzMechanismConstants.SHOOTER_DEFAULT_PRESET));

        xboxAux.a().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.SPEAKER_MODE).onTrue(shooter.rampUpFlyWheels());  //RAMPING UP 

        xboxAux.b().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.SPEAKER_MODE).onTrue(shooter.cmdShoot());                  //TO SHOOT (NEED TO RAMP UP FIRST)

        xboxAux.y().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.SPEAKER_MODE).onTrue(new AimAndOrFireAtSpeakerCmd());      //TO AUTO AIM TURRET+SERVOS TO SPEAKER 

        xboxAux.rightStick().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.SPEAKER_MODE).onTrue(shooter.cmdServoPosition(xboxAux.getRightY())); //MOVE SERVO POSITION MANUAL 

        xboxAux.rightStick().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.SPEAKER_MODE).onTrue(turret.rotate(xboxAux.getRightX()));            //MOVE TURRET POSITION MANUAL

    //------------------------------------------------------------------------------------
    // AMP MODE
    //------------------------------------------------------------------------------------              
        xboxDrv.leftStick().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.AMP_MODE).onTrue(new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.INTAKE_GROUND)); //DEPLOY INTAKE AND STOWS TO AMP SCORE DOWN POS

        xboxAux.leftTrigger().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.AMP_MODE).onTrue(new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER)); //NOTE IN SHOOTER TRANSFERED TO INTAKE

        xboxAux.y().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.AMP_MODE).onTrue(new MoveToAmpTransition());                      //MOVE TO AMP TRANSITION POSITION 

        xboxAux.b().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.AMP_MODE).onTrue(new ScoreAmpCmd());                              //SCORE AMP (^^ MUST BE IN AMP TRANSITION POS FIRST)

        xboxAux.leftStick().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.AMP_MODE).onTrue(new ManualElevatorCmd((()->xboxAux.getRightY()))); //MANUAL MODE FOR ELEVATOR


    //------------------------------------------------------------------------------------
    // HOARD MODE
    //------------------------------------------------------------------------------------
        // HOARD MODE IS NOT DONE BECAUSE WE STILL NEED TO WRITE CODE FOR IT 
        xboxAux.y().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.HOARD_MODE).onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND));  //PICK UP GROUND TO SHOOTER

        xboxAux.rightBumper().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.HOARD_MODE).onTrue(shooter.rampUpFlyWheels()); //RAMPING UP
          
        xboxAux.b().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.HOARD_MODE).onTrue(shooter.cmdShoot());                  //TO SHOOT (NEED TO RAMP UP FIRST)

        xboxAux.x().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.HOARD_MODE).onTrue(new AimAndOrFireAtSpeakerCmd());      //TO AUTO AIM TURRET+SERVOS TO SPEAKER

        xboxAux.a().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.HOARD_MODE).onTrue(intake.cmdRollerOut());               // INTAKE ROLLERS SHOOT



    //------------------------------------------------------------------------------------
    // CLIMB MODE
    //------------------------------------------------------------------------------------

        xboxAux.povUp().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.CLIMB_MODE).onTrue(new ClimbCmd(()-> xboxDrv.getLeftY(), ()-> xboxDrv.getRightY()));
        xboxAux.y().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.CLIMB_MODE).onTrue(new ScoreTrapCmd());
        xboxAux.b().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.CLIMB_MODE).onTrue(intake.cmdRollerOut());


    //------------------------------------------------------------------------------------
    // CLIMB MAINTANANCE MODE
    //------------------------------------------------------------------------------------
        xboxAux.povUp().and(()->CatzConstants.CatzMechanismConstants.driverCurrentMode == RobotMode.CLIMB_MAINTENANCE_MODE).onTrue(new ClimbCmd(()-> xboxDrv.getLeftY(), ()-> xboxDrv.getRightY()));

      
    //------------------------------------------------------------------------------------
    //  CHANGING MODES
    //------------------------------------------------------------------------------------
        xboxAux.povUp().and(xboxDrv.povUp()).onTrue(Commands.runOnce(()-> CatzConstants.CatzMechanismConstants.driverCurrentMode = RobotMode.CLIMB_MODE)); // CLIMB MODE

        xboxAux.rightBumper().and(xboxAux.leftBumper()).onTrue(Commands.runOnce(()->CatzConstants.CatzMechanismConstants.driverCurrentMode = RobotMode.CLIMB_MAINTENANCE_MODE)); //CLIMB MANTAINANCE MODE

        xboxAux.povDown().onTrue(Commands.runOnce(()->CatzConstants.CatzMechanismConstants.driverCurrentMode = RobotMode.HOARD_MODE));                     //HOARD MODE
      
        xboxAux.povLeft().onTrue(Commands.runOnce(()-> CatzConstants.CatzMechanismConstants.driverCurrentMode = RobotMode.AMP_MODE));                      //AMP MODE
      
        xboxAux.povRight().onTrue(Commands.runOnce(()->CatzConstants.CatzMechanismConstants.driverCurrentMode = RobotMode.SPEAKER_MODE));                  //SPEAKER MODE
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
