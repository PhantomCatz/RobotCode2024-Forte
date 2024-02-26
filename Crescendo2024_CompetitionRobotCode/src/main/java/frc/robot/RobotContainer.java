package frc.robot;

import com.ctre.phoenix6.mechanisms.MechanismState;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.ManipulatorMode;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.mechanismCmds.MoveToNewPositionCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.ManualIntakeCmd;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
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
    private SubsystemCatzIntake intake;
    //private SubsystemCatzShooter shooter;
    //private SubsystemCatzClimb climb;
    private SubsystemCatzElevator elevator;

    private CatzAutonomous auton = new CatzAutonomous();

    //xbox controller
    public static CommandXboxController xboxDrv;
    private CommandXboxController xboxAux;
 
       
   /** The container for the robot. Contains subsystems, OI devices, and commands. 
    *    -since multiple classes are referencing these mechansims, 
    *     mechanisms are instantiated inside mechanism class(singleton)
    */
   public RobotContainer() {
    //instantiate subsystems
    elevator = SubsystemCatzElevator.getInstance();
    driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    //vision     = SubsystemCatzVision.getInstance();
    intake     = SubsystemCatzIntake.getInstance();

   // shooter    = SubsystemCatzShooter.getInstance();
    //  climb      = SubsystemCatzClimb.getInstance();
    //  arm        = SubsystemCatzElevator.getInstance();
    

     xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);

 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
  
   
   private void configureBindings() {    
    // xboxAux.rightBumper().onTrue(intake.cmdRollerIn());
    // xboxAux.leftBumper().onTrue(intake.cmdRollerOut()); 
    // //trigger object to store both buttons. If both buttons aren't pressed, stop rollers
    // Trigger rollersOffBinding = xboxAux.leftBumper().and (xboxAux.rightBumper());
    // rollersOffBinding.onTrue(intake.cmdRollerOff());

    // Trigger manualTrigger = new Trigger(()-> Math.abs(xboxAux.getLeftY()) > 0.1);
    // manualTrigger.onTrue(new ManualIntakeCmd(()->xboxAux.getLeftY()));
    
    // xboxAux.rightStick().onTrue(new ManualElevatorCmd(()->xboxAux.getRightY()));

    // xboxAux.start().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.POS_STOW));
    // xboxAux.a().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_HANDOFF));
    // xboxAux.y().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_SCORING_AMP));
    // xboxAux.x().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_SOURCE));
    // xboxAux.b().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_GROUND));

    //----------------------------------------------------------------------------------------
    //  DriveControls
    //----------------------------------------------------------------------------------------
    xboxDrv.start().onTrue(driveTrain.resetGyro());

    xboxDrv.leftStick().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.NOTE_POS_INTAKE_GROUND, CatzConstants.currentManipulatorMode));
    xboxDrv.leftStick().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.POS_STOW, CatzConstants.currentManipulatorMode)); //TBD fix 

    xboxDrv.rightBumper().onTrue(intake.cmdRollerIn());
    xboxDrv.leftBumper().onTrue(intake.cmdRollerOut());
    //xboxDrv.

    Trigger driveLeftJoyYTrigger = new Trigger(()->Math.abs(xboxDrv.getLeftY()) > 0.1);
    driveLeftJoyYTrigger.onTrue(new SequentialCommandGroup()); // need climb subsystem and command to send in xbox values TBD\

    Trigger driveRightJoyYTrigger = new Trigger(()->Math.abs(xboxDrv.getRightY()) > 0.1);
    driveRightJoyYTrigger.onTrue(new SequentialCommandGroup()); // need climb subsystem and command to send in xbox values TBD\


    //----------------------------------------------------------------------------------------
    //  AuxControls
    //----------------------------------------------------------------------------------------    
  
    //note state button mappings
    xboxAux.povLeft().onTrue(Commands.runOnce(()->CatzConstants.currentManipulatorMode = ManipulatorMode.AMP)); //default state
    xboxAux.povUp().onTrue(Commands.runOnce(()->CatzConstants.currentManipulatorMode = ManipulatorMode.CLIMB));
    xboxAux.povDown().onTrue(Commands.runOnce(()->CatzConstants.currentManipulatorMode = ManipulatorMode.HOARD));
    xboxAux.povRight().onTrue(Commands.runOnce(()->CatzConstants.currentManipulatorMode = ManipulatorMode.SPEAKER));

    //all buttons are configured to the AMP mode
    //command will use current manipulator mode to switch states within the command
    xboxAux.b().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.NOTE_POS_SCORING_AMP, CatzConstants.currentManipulatorMode));
    xboxAux.y().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.NOTE_POS_HANDOFF, CatzConstants.currentManipulatorMode));
    xboxAux.x().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.AUTO_ALIGN_WITH_SPEAKER, CatzConstants.currentManipulatorMode));

    //roller commands
    xboxAux.leftTrigger().onTrue(intake.cmdRollerIn());
    xboxAux.rightTrigger().onTrue(intake.cmdRollerOut());
    
    //semi manual elevator/intake commands
    Trigger auxLeftJoyYTrigger = new Trigger(()->Math.abs(xboxAux.getLeftY()) > 0.1);
    auxLeftJoyYTrigger.onTrue(new ManualElevatorCmd(()->xboxAux.getLeftY()));

    Trigger auxRightJoyYTrigger = new Trigger(()->Math.abs(xboxAux.getRightY()) > 0.1);
    auxRightJoyYTrigger.onTrue(new ManualIntakeCmd(()->xboxAux.getRightY()));



  
    //xboxAux.povRight().onTrue(new MoveToNewPositionCmd(CatzMechanismConstants.))

   }

   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() {  
      driveTrain
      .setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
                                                      ()-> xboxDrv.getLeftY(),
                                                      ()-> xboxDrv.getRightX(),
                                                      ()-> xboxDrv.getRightTriggerAxis(), 
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
}
