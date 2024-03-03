package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.Utils.CatzMechanismPosition;
// import frc.robot.commands.AutoAlignCmd;
// import frc.robot.commands.DriveCmds.TeleopDriveCmd;
// import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
// import frc.robot.commands.mechanismCmds.MoveToNewPositionCmd;
// import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
// import frc.robot.subsystems.elevator.SubsystemCatzElevator;
// import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
// import frc.robot.subsystems.vision.SubsystemCatzVision;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

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
    // private SubsystemCatzDrivetrain driveTrain; 
    // private SubsystemCatzVision vision;
    // private SubsystemCatzIntake intake;
    private SubsystemCatzShooter shooter;
    // private SubsystemCatzElevator elevator;
    private SubsystemCatzTurret turret;
    //private SubsystemCatzClimb climb;


    // private CatzAutonomous auton = new CatzAutonomous();

    //xbox controller
    private CommandXboxController xboxDrv;
    private CommandXboxController xboxAux;
 
       
   /** The container for the robot. Contains subsystems, OI devices, and commands. 
    *    -since multiple classes are referencing these mechansims, 
    *     mechanisms are instantiated inside mechanism class(singleton)
    */
   public RobotContainer() {
    //instantiate subsystems

    // driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    // vision     = SubsystemCatzVision.getInstance();
    // intake     = SubsystemCatzIntake.getInstance();
    shooter    = SubsystemCatzShooter.getInstance();
    //elevator = SubsystemCatzElevator.getInstance();
    turret = SubsystemCatzTurret.getInstance();
    //climb      = SubsystemCatzClimb.getInstance();

    
     xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);
 
     // Configure the trigger bindings and default cmds
    //  defaultCommands();
     configureBindings();
   }
 
  
   
   private void configureBindings() {


    /*_____________________________________________________________________________________________
    *
    * 
    * INTAKE CMDS
    * 
    *
    _______________________________________________________________________________________________*/

    // xboxDrv.rightBumper().onTrue(intake.cmdRollerIn());                             //intake in
    // xboxDrv.leftBumper().onTrue(intake.cmdRollerOut());                             //intake out

    // Trigger rollersOffBinding = xboxDrv.leftBumper().and(xboxDrv.rightBumper());
    // rollersOffBinding.onTrue(intake.cmdRollerOff());                                //stop rollers

    
    /*_____________________________________________________________________________________________
    *
    * 
    * TURRET CMDS
    * 
    *
    _______________________________________________________________________________________________*/

    xboxAux.leftTrigger().onTrue(turret.cmdTurretLT()).onFalse(turret.cmdTurretOff());     //Turn Left  (-)
    xboxAux.rightTrigger().onTrue(turret.cmdTurretRT()).onFalse(turret.cmdTurretOff());    //Turn right (+)
    xboxAux.a().onTrue(turret.cmdResetTurretPosition());  // Go to home position (0.0)
  
    //  xboxAux.b().onTrue(turret.cmdAutoRotate()).onFalse(turret.cmdTurretOff());             //Follow April Tags

    //  xboxAux.start().onTrue(turret.cmdTurretOff());                                         //Turn off 
    
    /*_____________________________________________________________________________________________
    *
    * 
    * SHOOTER CMDS
    * 
    *
    _______________________________________________________________________________________________*/
    xboxDrv.rightTrigger().onTrue(shooter.cmdShoot());    //shooter activation
    xboxDrv.x().onTrue(shooter.cmdShooterEnabled());
    xboxDrv.y().onTrue(shooter.loadDisabled());
    xboxDrv.leftTrigger().onTrue(shooter.loadBackward());
    //xboxDrv.leftBumper().onTrue(shooter.setPosition(0.25));
    xboxDrv.rightBumper().onTrue(shooter.cmdLoad());
    
    /*_____________________________________________________________________________________________
    *
    * 
    * INTAKE CMDS
    * 
    *
    _______________________________________________________________________________________________*/

    // xboxAux.start().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.POS_STOW));

    // xboxAux.a().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_HANDOFF));
    // xboxAux.y().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_SCORING_AMP));
    // xboxAux.x().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_SOURCE));
    // xboxAux.b().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_GROUND));

    
    /*_____________________________________________________________________________________________
    *
    * 
    * ELEVATOR CMDS
    * 
    *
    _______________________________________________________________________________________________*/
    
    // xboxAux.rightStick().onTrue(new ManualElevatorCmd(()-> xboxAux.getRightY()));

    /*_____________________________________________________________________________________________
    *
    * 
    * IDK CMDS
    * 
    *
    _______________________________________________________________________________________________*/

    //xboxDrv.a().onTrue(auton.flyTrajectoryOne());
    //xboxDrv.back().onTrue(driveTrain.toggleVisionEnableCommand());
    // xboxDrv.start().onTrue(driveTrain.flipGyro());
    //xboxDrv.start().onTrue(driveTrain.resetGyro()); //classic gyro 0'ing 

    // xboxDrv.b().onTrue(driveTrain.stopDriving()); //TBD need to add this back in TBD runs when disabled where?

    //shooter activation
    //xboxDrv.x().onTrue(shooter.setShooterActive())
    //          .onFalse(shooter.setShooterDisabled());
    //xboxAux.rightBumper().onTrue(intake.setRollerIn()).onFalse(intake.setRollerDisabled());
    //xboxAux.leftBumper().onTrue(intake.setRollerOut()).onFalse(intake.setRollerDisabled());
    // xboxAux.a().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_SCORING_AMP));

    // Trigger intakePivotOverride = xboxAux.axisGreaterThan((int) (xboxAux.getLeftY()*100), 10);
    // intakePivotOverride.onTrue(intake.intakePivotOverrideCommand(xboxAux.getLeftY()))
    //                    .onFalse(intake.intakePivotOverrideCommand(0));

    // //xboxDrv.a().onTrue(auton.flyTrajectoryOne());
    // xboxDrv.back().onTrue(driveTrain.toggleVisionEnableCommand());
    // // xboxDrv.start().onTrue(driveTrain.flipGyro());
    // xboxDrv.start().onTrue(driveTrain.resetGyro()); //classic gyro 0'ing 

    // xboxDrv.b().onTrue(driveTrain.stopDriving()); //TBD need to add this back in TBD runs when disabled where?
    

    

  }

   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
  //  private void defaultCommands() {  
  //     driveTrain.setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
  //                                                     ()-> xboxDrv.getLeftY(),
  //                                                     ()-> xboxDrv.getRightX(),
  //                                                     ()-> xboxDrv.b().getAsBoolean()));
    
  //  }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    // public Command getAutonomousCommand() {
    //   return auton.getCommand();
    // }
}
