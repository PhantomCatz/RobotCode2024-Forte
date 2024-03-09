package frc.robot;

import com.ctre.phoenix6.mechanisms.MechanismState;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.mechanismCmds.MoveToHandoffPoseCmd;
import frc.robot.commands.mechanismCmds.AimAtSpeakerCmd;
import frc.robot.commands.mechanismCmds.ScoreTrapCmd;
import frc.robot.commands.mechanismCmds.StowCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.ManualIntakeCmd;
import frc.robot.subsystems.climb.SubsystemCatzClimb;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.vision.SubsystemCatzVision;

/**
 * RobotContainer
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is how the command scheduler(robot.java replacment) is configured
 * Configures:
 * -xbox controller triggers
 * -default commands
 * -instanciated mechanisms using singleton implementation
 * -sets up autonomous from CatzAtutonomouschooser
 */
 public class RobotContainer {
    
    //subsystems
    private SubsystemCatzDrivetrain driveTrain; 
    //private SubsystemCatzVision vision;
    private SubsystemCatzIntake     intake;
    private SubsystemCatzShooter  shooter;
    private SubsystemCatzClimb    climb;
    private SubsystemCatzElevator   elevator;
    private SubsystemCatzTurret     turret;

    private CatzAutonomous auton = new CatzAutonomous();

    //xbox controller
    public static CommandXboxController xboxDrv;
    private CommandXboxController       xboxAux;
 
       
   /** The container for the robot. Contains subsystems, OI devices, and commands. 
    *    -since multiple classes are referencing these mechansims, 
    *     mechanisms are instantiated inside mechanism class(singleton)
    */
   public RobotContainer() {
    //instantiate subsystems
    elevator   = SubsystemCatzElevator.getInstance();
    driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    //vision   = SubsystemCatzVision.getInstance();
    intake     = SubsystemCatzIntake.getInstance();
    turret     = SubsystemCatzTurret.getInstance();
    shooter    = SubsystemCatzShooter.getInstance();
    climb      = SubsystemCatzClimb.getInstance();
    

     xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);

 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
  
   
   private void configureBindings() {    
    
  //   xboxDrv.rightBumper().onTrue(intake.cmdRollerIn()); //TBD add in logic to run rollers on deply
  //   xboxDrv.leftBumper().onTrue(intake.cmdRollerOut()); 
  //   //trigger object to store both buttons. If both buttons aren't pressed, stop rollers

  //  // Trigger manualTrigger = new Trigger(()-> Math.abs(xboxDrv.getLeftY()) > 0.1);
  //   xboxDrv.leftStick().onTrue(new ManualIntakeCmd(()->xboxDrv.getLeftY(), ()->xboxDrv.leftStick().getAsBoolean()));
    
  //   xboxDrv.rightStick().onTrue(new ManualElevatorCmd(()->xboxDrv.getRightY(), ()->xboxDrv.rightStick().getAsBoolean()));

  //   xboxDrv.back().onTrue(shooter.cmdShoot());

  //   xboxDrv.start().onTrue(new StowCmd());
  //   xboxDrv.y().onTrue(new MoveToHandoffPoseCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND));
  //   xboxDrv.x().onTrue(new MoveToHandoffPoseCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER));
  //   xboxDrv.b().onTrue(new MoveToHandoffPoseCmd(NoteDestination.SPEAKER, NoteSource.FROM_INTAKE));

    //------------------------------------------------------------------------------------
    //  Drive commands
    //------------------------------------------------------------------------------------
    xboxDrv.leftStick().onTrue(new MoveToHandoffPoseCmd(targetNoteDestination, NoteSource.INTAKE_GROUND)); //intake pivot to ground
    xboxDrv.rightStick().onTrue(new StowCmd()); //intake pivot stow

    xboxDrv.start().onTrue(driveTrain.resetGyro());
    xboxDrv.povCenter().onTrue(new MoveToHandoffPoseCmd(targetNoteDestination, NoteSource.INTAKE_SOURCE));
    //upload button doesnt exist but its for source intake position

    xboxDrv.rightBumper().onTrue(intake.cmdRollerIn());
    xboxDrv.leftBumper().onTrue(intake.cmdRollerOut());
    Trigger rollersOffBinding = xboxDrv.leftBumper().and (xboxDrv.rightBumper());
    rollersOffBinding.onTrue(intake.cmdRollerOff());

    //Those commands should only work when the button is held
    xboxDrv.povUp().whileTrue(/*Both Climb hooks up*/null);
    xboxDrv.povDown().whileTrue(/*Both Climb hooks down*/null);
    xboxDrv.povRight().whileTrue(/*Raise Right hook*/null);
    xboxDrv.povLeft().whileTrue(/*Raise Left hook*/null);

    /*************************************************************
    * All Mode Buttons
    **************************************************************/
    //Default mode will be speaker mode when booting up

    //----------------------------------------------------------------------------------------
    //  State machine
    //----------------------------------------------------------------------------------------  
    xboxAux.povRight().onTrue(Commands.runOnce(()->targetNoteDestination = NoteDestination.SPEAKER));
    xboxAux.povLeft ().onTrue(Commands.runOnce(()->targetNoteDestination = NoteDestination.AMP));
    xboxAux.povUp   ().onTrue(Commands.runOnce(()->targetNoteDestination = NoteDestination.TRAP));
    xboxAux.povDown ().onTrue(Commands.runOnce(()->targetNoteDestination = NoteDestination.HOARD));
    
    
    //intake to shooter handoff
    xboxAux.y().onTrue(new MoveToHandoffPoseCmd(targetNoteDestination, NoteSource.FROM_INTAKE));
    //Shooter to intake handoff
    xboxAux.a().onTrue(new MoveToHandoffPoseCmd(targetNoteDestination, NoteSource.FROM_SHOOTER));

    xboxAux.a().and(xboxAux.b()).onTrue(new ScoreTrapCmd(()->xboxAux.povUp().getAsBoolean(), 
                                                         ()->xboxDrv.povDown().getAsBoolean(), 
                                                         ()->xboxAux.povLeft().getAsBoolean(), 
                                                         ()->xboxAux.povRight().getAsBoolean()));

    xboxAux.back().onTrue(/*Signify Amp LEDs*/null);
    // turn middle lights to red

    //shooter
    xboxAux.b().onTrue(shooter.cmdShoot());
    xboxAux.x().onTrue(new AimAtSpeakerCmd());
    xboxAux.leftStick().onTrue(shooter.cmdServoPosition(xboxAux.getLeftY()));
    xboxAux.rightStick().onTrue(shooter.cmdShooterRamp());

    //turret
    xboxAux.leftTrigger().onTrue(turret.cmdTurretLT());
    xboxAux.rightTrigger().onTrue(turret.cmdTurretRT());

    //intake
    xboxAux.leftBumper().onTrue(intake.cmdRollerIn());
    xboxAux.rightBumper().onTrue(intake.cmdRollerOut());
    Trigger rollersOffBindingAux = xboxAux.leftBumper().and(xboxAux.rightBumper());
    rollersOffBindingAux.onTrue(intake.cmdRollerOff());

    //Manual Commands
    xboxAux.leftStick().onTrue(new ManualElevatorCmd(()->xboxAux.getLeftY(), ()->xboxAux.leftStick().getAsBoolean()));
    xboxAux.rightStick().onTrue(new ManualIntakeCmd(()->xboxAux.getRightY(), ()->xboxAux.rightStick().getAsBoolean()));
  }

   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() {  
      driveTrain
      .setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
                                            ()-> xboxDrv.getLeftY(),
                                            ()-> xboxDrv.getRightX(),
                                            ()-> xboxDrv.b().getAsBoolean()));
    
  }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
  public Command getAutonomousCommand() {
    return auton.getCommand();
  }

  private static NoteDestination targetNoteDestination;
  public static enum NoteDestination {
    SPEAKER,
    AMP,
    TRAP,
    HOARD
  }
  public static enum NoteSource {
    INTAKE_SOURCE,
    INTAKE_GROUND,
    FROM_SHOOTER,
    FROM_INTAKE,
    NULL
  }
}
