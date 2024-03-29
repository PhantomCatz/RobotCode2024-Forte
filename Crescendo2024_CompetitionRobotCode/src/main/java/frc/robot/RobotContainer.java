package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.CatzColorConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.CatzConstants.RobotMode;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.mechanismCmds.MoveToPresetHandoffCmd;
import frc.robot.commands.mechanismCmds.MoveToPreset;
import frc.robot.commands.mechanismCmds.ScoreAmpCmd;
import frc.robot.commands.mechanismCmds.ScoreTrapCmd;
import frc.robot.commands.mechanismCmds.ClimbCmd;
import frc.robot.commands.mechanismCmds.StowPoseCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.AimAndOrFireAtSpeakerCmd;
import frc.robot.CatzConstants.NoteDestination;
import frc.robot.CatzConstants.NoteSource;
import frc.robot.subsystems.LEDs.SubsystemCatzLED;
import frc.robot.subsystems.climb.SubsystemCatzClimb;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterState;
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


  private CatzAutonomous auton;

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

    auton     = CatzAutonomous.getInstance();
    

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
   
    //------------------------------------------------------------------------------------
    // POSE COMMANDS
    //------------------------------------------------------------------------------------
      xboxDrv.rightStick().onTrue(new StowPoseCmd()); //STOW Mechnanism 
      xboxAux.rightStick().onTrue(new StowPoseCmd()); //STOW Mechnanism 

    
      
    //LED command
    xboxAux.back().onTrue(Commands.runOnce(()-> led.signalHumanPlayerAMP())); // SIGNAL HUMAN PLAYER FOR AMP SCORING


    //------------------------------------------------------------------------------------
    // SPEAKER MODE
    //------------------------------------------------------------------------------------ 
        Trigger triggerModeSpeaker = new Trigger(()->isInSpeakerMode());

        triggerModeSpeaker.and(xboxAux.leftTrigger())
                          .onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)); //DEPLOY INTAKE & STOWS & STORES TO SHOOTER
                    
        triggerModeSpeaker.and(xboxAux.leftTrigger())
                          .onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.FROM_INTAKE));//NOTE IN INTAKE TRANSFER TO SHOOTER

        triggerModeSpeaker.and(xboxAux.x())
                          .onTrue(new MoveToPreset(CatzMechanismConstants.SHOOTER_DEFAULT_PRESET));

        triggerModeSpeaker.and(xboxAux.a())
                          .onTrue(Commands.runOnce(()->shooter.setShooterState(ShooterState.START_SHOOTER_FLYWHEEL)));  //RAMPING UP 

        triggerModeSpeaker.and(xboxAux.b())
                          .onTrue(Commands.runOnce(()->shooter.setShooterState(ShooterState.SHOOTING)));  //TO SHOOT (NEED TO RAMP UP FIRST)

        triggerModeSpeaker.and(xboxAux.y())
                          .onTrue(new AimAndOrFireAtSpeakerCmd());      //TO AUTO AIM TURRET+SERVOS TO SPEAKER 

        Trigger auxJoystickTriggerRightX = new Trigger(()->xboxAux.getLeftY() > 0.1);
        auxJoystickTriggerRightX.and(()->isInSpeakerMode()).onTrue(shooter.cmdManualHoldOn(()->-xboxAux.getLeftY())); //MOVE SERVO POSITION MANUAL 

        Trigger auxJoystickTriggerRightY = new Trigger(()->xboxAux.getRightX() > 0.1);
        auxJoystickTriggerRightY.and(()->isInSpeakerMode()).onTrue(turret.cmdRotateTurretManualOn(()->xboxAux.getRightX()));            //MOVE TURRET POSITION MANUAL

        
    //------------------------------------------------------------------------------------
    // AMP MODE
    //------------------------------------------------------------------------------------  
        Trigger triggerModeAmp = new Trigger(()->isInAmpMode());
            
        triggerModeAmp.and(xboxDrv.leftStick())
                      .onTrue(new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.INTAKE_GROUND)); //DEPLOY INTAKE AND STOWS TO AMP SCORE DOWN POS

        triggerModeAmp.and(xboxAux.leftTrigger())
                      .onTrue(new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER)); //NOTE IN SHOOTER TRANSFERED TO INTAKE

        triggerModeAmp.and(xboxAux.b())
                      .onTrue(new MoveToPreset(CatzMechanismConstants.SCORING_AMP_PRESET));                              //SCORE AMP (^^ MUST BE IN AMP TRANSITION POS FIRST)
        
        triggerModeAmp.and(xboxAux.leftStick())
                      .onTrue(new ManualElevatorCmd((()->xboxAux.getRightY()))); //MANUAL MODE FOR ELEVATOR


    //------------------------------------------------------------------------------------
    // HOARD MODE
    //------------------------------------------------------------------------------------

        Trigger triggerModeHoard = new Trigger(()->isInHoardMode());

        triggerModeHoard.and(xboxAux.y())
                        .onTrue(shooter.hoardShooterShot());  //MOVES TURRET/SERVOS TO CORRECT POS + RAMPS UP SHOOTER
          
        triggerModeHoard.and(xboxAux.b())
                        .onTrue(shooter.cmdShoot());          //TO SHOOT (NEED TO RAMP UP FIRST)

        triggerModeHoard.and(xboxAux.x())
                        .onTrue(new MoveToPreset(CatzMechanismConstants.INTAKE_HOARD_PRESET));      //TO HOARD INTAKE POS

        triggerModeHoard.and(xboxAux.a())
                        .onTrue(intake.cmdRollerOut());        // INTAKE ROLLERS SHOOT

        triggerModeHoard.and(xboxAux.leftBumper())
                        .onTrue(new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER));     //NOTE IN SHOOTER TRANSFERED TO INTAKE
        triggerModeHoard.and(xboxAux.rightBumper())
                        .onTrue(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.FROM_INTAKE)); //NOTE IN INTAKE TRANSFERED TO SHOOTER

    //------------------------------------------------------------------------------------  
    // CLIMB MODE
    //------------------------------------------------------------------------------------

        xboxAux.povUp().and(()->isInClimbMode()).onTrue(new ClimbCmd(()-> xboxDrv.getLeftY(), ()-> xboxDrv.getRightY()));
        xboxAux.y().and(()->isInClimbMode()).onTrue(new ScoreTrapCmd());
        xboxAux.b().and(()->isInClimbMode()).onTrue(intake.cmdRollerOut());


    //------------------------------------------------------------------------------------
    // CLIMB MAINTANANCE MODE
    //------------------------------------------------------------------------------------
        xboxAux.povUp().and(()->CatzConstants.currentRobotMode == RobotMode.CLIMB_MAINTENANCE_MODE).onTrue(new ClimbCmd(()-> xboxDrv.getLeftY(), ()-> xboxDrv.getRightY()));

      
    //------------------------------------------------------------------------------------
    //  CHANGING MODES
    //------------------------------------------------------------------------------------
        xboxAux.povUp().and(xboxDrv.povUp()).onTrue(Commands.runOnce(()-> CatzConstants.currentRobotMode = RobotMode.CLIMB)); // CLIMB MODE

        xboxAux.rightBumper().and(xboxAux.leftBumper()).onTrue(Commands.runOnce(()->CatzConstants.currentRobotMode = RobotMode.CLIMB_MAINTENANCE_MODE)); //CLIMB MANTAINANCE MODE

        xboxAux.povDown().onTrue(Commands.runOnce(()->CatzConstants.currentRobotMode = RobotMode.HOARD));                     //HOARD MODE
      
        xboxAux.povLeft().onTrue(Commands.runOnce(()->CatzConstants.currentRobotMode = RobotMode.AMP));                      //AMP MODE
      
        xboxAux.povRight().onTrue(Commands.runOnce(()->CatzConstants.currentRobotMode = RobotMode.SPEAKER));                  //SPEAKER MODE
    //------------------------------------------------------------------------------------
  }

  public void logDpadStates() {
       SmartDashboard.putString("Scoring Mode", CatzConstants.currentRobotMode.toString());

  }

  private boolean isInHoardMode() {
    return CatzConstants.currentRobotMode == RobotMode.HOARD;
  }

  private boolean isInClimbMode() {
    return CatzConstants.currentRobotMode == RobotMode.CLIMB;
  }

  private boolean isInSpeakerMode() {
    return CatzConstants.currentRobotMode == RobotMode.SPEAKER;
  }

  private boolean isInAmpMode() {
    return CatzConstants.currentRobotMode == RobotMode.AMP;
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
