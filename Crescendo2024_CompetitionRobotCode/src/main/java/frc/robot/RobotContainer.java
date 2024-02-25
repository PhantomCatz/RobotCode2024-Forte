package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.OIConstants;
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
 */
 public class RobotContainer {
    //subsystems
    //private SubsystemCatzDrivetrain driveTrain; 
    private SubsystemCatzVision vision;
    //private SubsystemCatzIntake intake;
    private SubsystemCatzShooter shooter;
    //private SubsystemCatzClimb climb;
    private SubsystemCatzElevator elevator;

    private CatzAutonomous auton = new CatzAutonomous();

    //xbox controller
    public static CommandXboxController xboxDrv;
    private CommandXboxController xboxAux;
 
       
   public RobotContainer() {
    //instantiate subsystems
    elevator = SubsystemCatzElevator.getInstance();
    driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    //vision     = SubsystemCatzVision.getInstance();
    //intake     = SubsystemCatzIntake.getInstance();

    shooter    = SubsystemCatzShooter.getInstance();
    elevator = SubsystemCatzElevator.getInstance();
    //  climb      = SubsystemCatzClimb.getInstance();
    //  arm        = SubsystemCatzElevator.getInstance();
    

     xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);
 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
   
   private void configureBindings() { 
  
  
      xboxDrv.rightTrigger().onTrue(shooter.cmdShoot());    //shooter activation
      xboxDrv.x().onTrue(shooter.cmdShooterEnabled());
                 //.onFalse(shooter.cmdShooterDisabled());
      xboxDrv.y().onTrue(shooter.loadDisabled());
      xboxDrv.leftTrigger().onTrue(shooter.loadBackward());
      xboxDrv.leftBumper().onTrue(shooter.setPosition(0.25));

      xboxDrv.rightBumper().onTrue(shooter.cmdLoad());   

      xboxAux.rightBumper().onTrue(intake.cmdRollerIn());
      xboxAux.leftBumper().onTrue(intake.cmdRollerOut()); 
     
      //trigger object to store both buttons. If both buttons aren't pressed, stop rollers
      Trigger rollersOffBinding = xboxAux.leftBumper().and (xboxAux.rightBumper());
      rollersOffBinding.onTrue(intake.cmdRollerOff());
      Trigger manualTrigger = new Trigger(()-> Math.abs(xboxAux.getLeftY()) > 0.1);
      manualTrigger.onTrue(new ManualIntakeCmd(()->xboxAux.getLeftY()));

      xboxAux.rightStick().onTrue(new ManualElevatorCmd(()->xboxAux.getRightY()));

      xboxAux.start().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.POS_STOW));
      xboxAux.a().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_HANDOFF));
      xboxAux.y().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_SCORING_AMP));
      xboxAux.x().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_SOURCE));
      xboxAux.b().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_GROUND));

      xboxDrv.a().onTrue(auton.autoFindPathSource());
      xboxDrv.back().onTrue(driveTrain.toggleVisionEnableCommand());
      xboxDrv.start().onTrue(driveTrain.resetGyro()); //classic gyro 0'ing 
 

   }

   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() {  
      driveTrain.setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(), //tranlate X
                                                      ()-> xboxDrv.getLeftY(), //translate Y
                                                      ()-> xboxDrv.getRightX(), //rotation thetha
                                                      ()-> xboxDrv.b().getAsBoolean())); //determine if chassis is field oriented or no
   }

   //autonomous collection
  public Command getAutonomousCommand() {
    return auton.getCommand();
  }
}
