package frc.robot;

import com.ctre.phoenix6.mechanisms.MechanismState;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.NoteDestination;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.AutoAlignCmd;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.mechanismCmds.MoveToNewPositionCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.ManualIntakeCmd;
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
    //private SubsystemCatzClimb    climb;
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
    //  climb      = SubsystemCatzClimb.getInstance();
    //  arm        = SubsystemCatzElevator.getInstance();
    

     xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);

 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
  
   
   private void configureBindings() {    
    
    xboxDrv.rightBumper().onTrue(intake.cmdRollerIn()); //TBD add in logic to run rollers on deply
    xboxDrv.leftBumper().onTrue(intake.cmdRollerOut()); 
    //trigger object to store both buttons. If both buttons aren't pressed, stop rollers
   Trigger rollersOffBinding = xboxDrv.leftBumper().and (xboxDrv.rightBumper());
    rollersOffBinding.onTrue(intake.cmdRollerOff());

   // Trigger manualTrigger = new Trigger(()-> Math.abs(xboxDrv.getLeftY()) > 0.1);
    xboxDrv.leftStick().onTrue(new ManualIntakeCmd(()->xboxDrv.getLeftY(), ()->xboxDrv.leftStick().getAsBoolean()));
    
    xboxDrv.rightStick().onTrue(new ManualElevatorCmd(()->xboxDrv.getRightY(), ()->xboxDrv.rightStick().getAsBoolean()));

    xboxDrv.back().onTrue(shooter.cmdShoot());

    xboxDrv.back().onTrue(driveTrain.resetGyro());

    xboxDrv.start().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.POS_STOW, ()->CatzConstants.targetNoteDestination));
    xboxDrv.y().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_SCORING_AMP, ()->CatzConstants.targetNoteDestination));
    xboxDrv.x().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_SOURCE, ()->CatzConstants.targetNoteSource));
    xboxDrv.b().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_GROUND, ()->CatzConstants.targetNoteSource));

    // xboxDrv.leftBumper().onTrue(shooter.cmdLoad());
    // xboxDrv.rightBumper().onTrue(shooter.loadBackward()).onFalse(shooter.loadDisabled());


    // xboxDrv.leftTrigger().onTrue(turret.cmdTurretLT()).onFalse(turret.cmdTurretOff());
    // xboxDrv.rightTrigger().onTrue(turret.cmdTurretRT()).onFalse(turret.cmdTurretOff());

  //   //----------------------------------------------------------------------------------------
  //   //  DriveControls
  //  //----------------------------------------------------------------------------------------
  //   xboxDrv.start().onTrue(driveTrain.resetGyro());

  //   xboxDrv.leftStick().onTrue(new ParallelCommandGroup(new MoveToNewPositionCmd(CatzMechanismConstants.NOTE_POS_INTAKE_GROUND, ()->CatzConstants.currentManipulatorMode),
  //                                                       intake.cmdRollerIn()));
  //   xboxDrv.leftStick().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.POS_STOW, ()->CatzConstants.currentManipulatorMode)); //TBD fix 

  //   xboxDrv.rightBumper().onTrue(intake.cmdRollerIn());
  //   xboxDrv.leftBumper().onTrue(intake.cmdRollerOut());
  //  Trigger rollersOffBindingDrv = xboxDrv.leftBumper().and (xboxDrv.rightBumper());
  //   rollersOffBindingDrv.onTrue(intake.cmdRollerOff());


  //   //xboxDrv.

  //   Trigger driveLeftJoyYTrigger = new Trigger(()->Math.abs(xboxDrv.getLeftY()) > 0.1);
  //   driveLeftJoyYTrigger.onTrue(new SequentialCommandGroup()); // need climb subsystem and command to send in xbox values TBD\

  //   Trigger driveRightJoyYTrigger = new Trigger(()->Math.abs(xboxDrv.getRightY()) > 0.1);
  //   driveRightJoyYTrigger.onTrue(new SequentialCommandGroup()); // need climb subsystem and command to send in xbox values TBD\


    // // ----------------------------------------------------------------------------------------
    // //  AuxControls
    // // ----------------------------------------------------------------------------------------    

    // //all buttons are configured to the AMP mode
    // //command will use current manipulator mode to switch states within the command
    // xboxAux.b().onTrue(new ParallelCommandGroup(new MoveToNewPositionCmd(CatzMechanismConstants.NOTE_SCORING_AMP,()-> CatzConstants.currentManipulatorMode),
    //                                             shooter.cmdShoot()));
    // xboxAux.y().onTrue(new ParallelCommandGroup(new MoveToNewPositionCmd(CatzMechanismConstants.NOTE_POS_HANDOFF_SPEAKER_PREP, ()->CatzConstants.currentManipulatorMode),
    //                                             new AutoAlignCmd()));
                                                
    // xboxAux.a().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.NOTE_POS_HANDOFF_AMP_PREP, ()->CatzConstants.currentManipulatorMode));
    // xboxAux.x().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.AUTO_ALIGN_WITH_SPEAKER, ()->CatzConstants.currentManipulatorMode));

    // xboxAux.start().onTrue(new SequentialCommandGroup()); ///signify amp leds

    // xboxAux.rightTrigger().onTrue(intake.cmdRollerIn());
    // xboxAux.leftTrigger().onTrue(intake.cmdRollerOut()); 
    // //trigger object to store both buttons. If both buttons aren't pressed, stop rollers
    // Trigger rollersOffBinding = xboxAux.leftTrigger().and (xboxDrv.rightTrigger());
    // rollersOffBinding.onTrue(intake.cmdRollerOff());

    // //roller commands
    // xboxAux.leftTrigger().onTrue(intake.cmdRollerIn());
    // xboxAux.rightTrigger().onTrue(intake.cmdRollerOut());
    
    // //semi manual elevator/intake commands
    // Trigger auxLeftJoyYTrigger = new Trigger(()->Math.abs(xboxAux.getLeftY()) > 0.1);
    // auxLeftJoyYTrigger.onTrue(new ManualElevatorCmd(()->xboxAux.getLeftY(), ()->xboxAux.leftStick().getAsBoolean()));

    // Trigger auxRightJoyYTrigger = new Trigger(()->Math.abs(xboxAux.getRightY()) > 0.1);
    // auxRightJoyYTrigger.onTrue(new ManualIntakeCmd(()->xboxAux.getRightY(), ()->xboxAux.rightStick().getAsBoolean()));

    //----------------------------------------------------------------------------------------
    //  State machine
    //---------------------------------------------------------------------------------------- 
    xboxDrv.povRight().onTrue(Commands.runOnce(()->CatzConstants.targetNoteDestination = NoteDestination.SPEAKER));
    xboxDrv.povLeft().onTrue(Commands.runOnce(()->CatzConstants.targetNoteDestination = NoteDestination.AMP));
    xboxDrv.povUp().onTrue(Commands.runOnce(()->CatzConstants.targetNoteDestination = NoteDestination.TRAP)); //Climber

    //note state button mappings
    xboxAux.povLeft().onTrue(Commands.runOnce(()->CatzConstants.targetNoteDestination = NoteDestination.AMP)); //default state
    xboxAux.povUp().onTrue(Commands.runOnce(()->CatzConstants.targetNoteDestination = NoteDestination.TRAP));
    xboxAux.povDown().onTrue(Commands.runOnce(()->CatzConstants.targetNoteDestination = NoteDestination.HOARD));
    xboxAux.povRight().onTrue(Commands.runOnce(()->CatzConstants.targetNoteDestination = NoteDestination.SPEAKER));

  }

   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() {  
      // driveTrain
      // .setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
      //                                       ()-> xboxDrv.getLeftY(),
      //                                       ()-> xboxDrv.getRightX(),
      //                                       ()-> xboxDrv.b().getAsBoolean()));
    
  }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
  public Command getAutonomousCommand() {
    return auton.getCommand();
  }
}
