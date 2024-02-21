package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import frc.robot.commands.StateMachineCmds.MoveToNewPositionCmd;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
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
    private SubsystemCatzDrivetrain driveTrain; 
    private SubsystemCatzVision vision;
    //private SubsystemCatzShooter shooter;
    //private SubsystemCatzClimb climb;
    //private SubsystemCatzElevator arm;

    private CatzAutonomous auton = new CatzAutonomous();

    //xbox controller
    private CommandXboxController xboxDrv;
    private CommandXboxController xboxAux;
 
       
   public RobotContainer() {
    //instantiate subsystems
    driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    vision     = SubsystemCatzVision.getInstance();

    //shooter    = SubsystemCatzShooter.getInstance();
    //  climb      = SubsystemCatzClimb.getInstance();
    //  arm        = SubsystemCatzElevator.getInstance();
    

     xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);
 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
   //bind mapping
   private void configureBindings() {

    xboxAux.a().onTrue(new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_SCORING_AMP));
    
    xboxDrv.a().onTrue(auton.autoFindPathSource());
    xboxDrv.back().onTrue(driveTrain.toggleVisionEnableCommand());
    xboxDrv.start().onTrue(driveTrain.resetGyro()); //classic gyro 0'ing 
 
   }

   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() {  
      driveTrain.setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(), //tranlate X
                                                      ()-> xboxDrv.getLeftY(), //translate Y
                                                      ()-> xboxDrv.getRightX(), //rotation thetha
                                                      ()-> xboxDrv.b().getAsBoolean())); //determine if chassis is field oriented or not
   }

   //autonomous collection
  public Command getAutonomousCommand() {
    return auton.getCommand();
  }
}
