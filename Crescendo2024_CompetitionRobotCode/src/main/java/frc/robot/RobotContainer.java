package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.mechanismCmds.MoveToHandoffPoseCmd;
import frc.robot.commands.mechanismCmds.ScoreAmpCmd;
import frc.robot.commands.mechanismCmds.ScoreTrapCmd;
import frc.robot.commands.mechanismCmds.StowCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.IntakeManualCmd;
import frc.robot.subsystems.CatzStateMachine;
import frc.robot.subsystems.CatzStateMachine.NoteDestination;
import frc.robot.subsystems.CatzStateMachine.NoteSource;
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
  //private SubsystemCatzVision vision;
  private SubsystemCatzIntake     intake;
  private SubsystemCatzShooter  shooter;
  private SubsystemCatzClimb    climb;
  private SubsystemCatzElevator   elevator;
  private SubsystemCatzTurret     turret;

  private CatzStateMachine stateMachine;

  private CatzAutonomous auton = new CatzAutonomous();

  //xbox controller
  private CommandXboxController xboxDrv;
  private CommandXboxController xboxAux;

  public RobotContainer() {
    //instantiate subsystems
    elevator   = SubsystemCatzElevator.getInstance();
    driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    //vision   = SubsystemCatzVision.getInstance();
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
    //------------------------------------------------------------------------------------
    //  Drive commands
    //------------------------------------------------------------------------------------
    xboxDrv.leftStick().onTrue(new MoveToHandoffPoseCmd(NoteDestination.AMP, NoteSource.INTAKE_GROUND)); //intake pivot to ground
    xboxDrv.start().onTrue(driveTrain.resetGyro());
    xboxDrv.rightStick().onTrue(new MoveToHandoffPoseCmd(NoteDestination.AMP, NoteSource.INTAKE_SOURCE));

    //intake to shooter handoff and vice versa
    xboxDrv.y().onTrue(stateMachine.cmdDetermineHandOffTransition());

    //climb
    xboxDrv.b().and(xboxAux.a()).onTrue(new ScoreTrapCmd(()->xboxAux.povUp().getAsBoolean(),      //both climb hooks up
                                                          ()->xboxAux.povDown().getAsBoolean(),    //both climb hooks down
                                                          ()->xboxAux.povLeft().getAsBoolean(),    //raise right climb hook
                                                          ()->xboxAux.povRight().getAsBoolean())); //raise left climb hook

    //----------------------------------------------------------------------------------------
    //  Aux Commands
    //---------------------------------------------------------------------------------------- 
    //pov state machine commands 
    xboxAux.povRight().onTrue(stateMachine.cmdNewNoteDestintation(NoteDestination.SPEAKER));
    xboxAux.povLeft ().onTrue(stateMachine.cmdNewNoteDestintation(NoteDestination.AMP));
    xboxAux.povUp   ().onTrue(stateMachine.cmdNewNoteDestintation(NoteDestination.TRAP));
    xboxAux.povDown ().onTrue(stateMachine.cmdNewNoteDestintation(NoteDestination.HOARD));


    //Shooter to intake handoff
    xboxAux.y().onTrue(stateMachine.cmdDetermineHandOffTransition());
    xboxAux.b().onTrue(stateMachine.cmdDetermineButtonBCommand(()->xboxAux.b().getAsBoolean()));
    xboxAux.a().onTrue(new StowCmd()); //intake pivot stow

    // xboxAux.back().onTrue(/*Signify Amp LEDs*/null);
    // turn middle lights to red

    //statmachine shooter vs intake elevator manual control dependant on state
    xboxAux.leftStick().onTrue(stateMachine.cmdDetermineAuxLeftSick  (()->xboxAux.getLeftY(), 
                                                                      ()->xboxAux.leftStick().getAsBoolean()));
    xboxAux.rightStick().onTrue(stateMachine.cmdDetermineAuxRightSick(()->xboxAux.getRightY(), 
                                                                      ()->xboxAux.rightStick().getAsBoolean()));

    //turret
    xboxAux.leftTrigger().onTrue(turret.cmdTurretLT());
    xboxAux.rightTrigger().onTrue(turret.cmdTurretRT());

    //intake
    xboxAux.leftBumper().onTrue(intake.cmdRollerIn());
    xboxAux.rightBumper().onTrue(intake.cmdRollerOut());
    Trigger rollersOffBindingAux = xboxAux.leftBumper().and(xboxAux.rightBumper());
    rollersOffBindingAux.onTrue(intake.cmdRollerOff());
  }

  //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
  private void defaultCommands() {  
    driveTrain.setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
                                                    ()-> xboxDrv.getLeftY(),
                                                    ()-> xboxDrv.getRightX(),
                                                    ()-> xboxDrv.b().getAsBoolean()));

  }

  public Command getAutonomousCommand() {
    return auton.getCommand();
  }

}
