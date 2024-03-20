package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.mechanismCmds.MoveToHandoffPoseCmd;
import frc.robot.commands.mechanismCmds.MoveToPreset;
import frc.robot.commands.mechanismCmds.ScoreAmpCmd;
import frc.robot.commands.mechanismCmds.ScoreTrapCmd;
import frc.robot.commands.mechanismCmds.ClimbCmd;
import frc.robot.commands.mechanismCmds.StowPoseCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.MoveToAmpTransition;
import frc.robot.commands.mechanismCmds.AimAndOrFireAtSpeakerCmd;
import frc.robot.commands.mechanismCmds.IntakeManualCmd;
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
    //xboxTest = new CommandXboxController(2);

    // Configure the trigger bindings and default cmds
    defaultCommands();
    configureBindings();
  }
  

  private void configureBindings() {    

    //xboxDrv.y().onTrue(turret.testTurretAngles()); //delete later
    
    //------------------------------------------------------------------------------------
    //  Drive commands
    //------------------------------------------------------------------------------------
    //mode speaker
    xboxDrv.leftStick().and(xboxAux.povRight()).onTrue(new MoveToHandoffPoseCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND));

    //mode amp
    xboxDrv.leftStick().and(xboxAux.povLeft()).onTrue(new MoveToHandoffPoseCmd(NoteDestination.AMP, NoteSource.INTAKE_GROUND));                               


    xboxDrv.start().onTrue(driveTrain.resetGyro());

    //ensure that the robot is shooter facing the speaker when reseting position
    xboxDrv.back().and(xboxDrv.leftTrigger()).onTrue(Commands.runOnce(()->driveTrain.resetPosition(new Pose2d(2.97,4.11, Rotation2d.fromDegrees(0)))));

    //signify amp
    xboxDrv.x().and(xboxDrv.back()).onTrue(Commands.runOnce(()->led.signalHumanPlayerAMP()));

    //----------------------------------------------------------------------------------------
    //  Aux Commands
    //---------------------------------------------------------------------------------------- 
    //pov state machine commands 

      //climb
    xboxAux.back().and(xboxAux.start()).onTrue(new ClimbCmd(()->xboxAux.getLeftY(), ()->xboxAux.getRightY()));//.onFalse(new ClimbCmd(()->0.0, ()->0.0));  //raise left climb hook 

    
    //mode speaker
    xboxAux.y().and(xboxAux.povRight()).onTrue(new MoveToHandoffPoseCmd(NoteDestination.SPEAKER, NoteSource.FROM_INTAKE));

    //xboxAux.x().and(xboxAux.povRight()).onTrue(new AimAndOrFireAtSpeakerCmd(()->xboxAux.b().getAsBoolean()));

    xboxAux.b().and(xboxAux.povRight()).onTrue(shooter.cmdShoot());

    xboxAux.rightStick().and(xboxAux.povRight()).onTrue(shooter.cmdShooterRamp());

    xboxAux.povDown().and(xboxAux.x()).onTrue(new MoveToPreset(CatzMechanismConstants.SCORING_TRAP_PRESET));
    xboxAux.povDown().and(xboxAux.b()).onTrue(new MoveToPreset(CatzMechanismConstants.INTAKE_SOURCE_PRESET));

    xboxAux.a().and(xboxAux.povUp()).onTrue(shooter.cmdServoPosition(1.0)); 
    xboxAux.a().and(xboxAux.povDown()).onTrue(shooter.cmdServoPosition(0.0)); 


    xboxAux.a().and(xboxAux.povRight()).onTrue(new MoveToPreset(CatzMechanismConstants.SUBWOOFER_PRESET));

    xboxAux.b().and(xboxAux.x()).and(xboxAux.povRight()).onTrue(new MoveToPreset(CatzMechanismConstants.SUBWOOFER_DEFENSE_PRESET));


    //mode amp
    xboxAux.y().and(xboxAux.povLeft()).onTrue(new MoveToHandoffPoseCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER));

    xboxAux.x().and(xboxAux.povLeft()).onTrue(new MoveToAmpTransition());

    xboxAux.b().and(xboxAux.povLeft()).onTrue(new ScoreAmpCmd());

    xboxAux.rightStick().onTrue(shooter.setPositionCmd(()->xboxAux.getRightY()));

    // xboxAux.start().onTrue(new MoveToPreset(CatzMechanismConstants.INTAKE_SOURCE));


    //mode trap
    xboxAux.povUp().and(xboxAux.x()).onTrue(new ScoreTrapCmd());

    //stow
    xboxAux.a().onTrue(new StowPoseCmd());

    


    // turn middle lights to red

    //statmachine shooter vs intake elevator manual control dependant on state
    xboxAux.leftStick().onTrue(new ManualElevatorCmd(()->xboxAux.getLeftY(), ()->xboxAux.leftStick().getAsBoolean()));

    //turret
    xboxAux.leftTrigger().onTrue(turret.cmdTurretLT()).onFalse(turret.cmdTurretOff());
    xboxAux.rightTrigger().onTrue(turret.cmdTurretRT()).onFalse(turret.cmdTurretOff());


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
                                                    ()-> -xboxDrv.getRightX(),
                                                    ()-> xboxDrv.b().getAsBoolean()));

  }

  public Command getAutonomousCommand() {
    return auton.getCommand();
  }

}
