package frc.robot;

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
import frc.robot.CatzConstants.OIConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.AutoAlignCmd;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.mechanismCmds.MoveToNewPositionCmd;
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
    //private SubsystemCatzDrivetrain driveTrain; 
    private SubsystemCatzVision vision;
    //private SubsystemCatzIntake intake;
    private SubsystemCatzShooter shooter;
    //private SubsystemCatzClimb climb;
    private SubsystemCatzElevator elevator;
    private SubsystemCatzTurret turret;

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

    //driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    //vision     = SubsystemCatzVision.getInstance();
    //intake     = SubsystemCatzIntake.getInstance();

    shooter    = SubsystemCatzShooter.getInstance();
    elevator = SubsystemCatzElevator.getInstance();
    //  climb      = SubsystemCatzClimb.getInstance();
    //  arm        = SubsystemCatzElevator.getInstance();
     turret = SubsystemCatzTurret.getInstance();
    
     xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);
 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
  
   
   private void configureBindings() {

     xboxAux.leftTrigger().onTrue(turret.cmdTurretLT()).onFalse(turret.cmdTurretOff());
     xboxAux.rightTrigger().onTrue(turret.cmdTurretRT()).onFalse(turret.cmdTurretOff());
     xboxAux.a().onTrue(turret.cmdResetTurretPosition()).onFalse(turret.cmdTurretOff());
     xboxAux.x().onTrue(turret.cmdTurretDegree(0.0));
     xboxAux.b().onTrue(turret.cmdAutoRotate()).onFalse(turret.cmdTurretOff());

      xboxDrv.rightTrigger().onTrue(shooter.loadFowardCmd());    //shooter activation
      xboxDrv.x().onTrue(shooter.cmdShooterEnabled())
                .onFalse(shooter.cmdShooterDisabled());
      xboxDrv.y().onTrue(shooter.loadDisabled());
      xboxDrv.leftTrigger().onTrue(shooter.loadBackward());
      xboxDrv.leftBumper().onTrue(shooter.setServoPowerExtend());
 
   }

   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() {  
      // driveTrain.setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
      //                                                 ()-> xboxDrv.getLeftY(),
      //                                                 ()-> xboxDrv.getRightX(),
      //                                                 ()-> xboxDrv.getRightTriggerAxis(), 
      //                                                 ()-> xboxDrv.b().getAsBoolean()));
    
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
