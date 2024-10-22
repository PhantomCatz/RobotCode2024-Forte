// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Locale.Category;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.revrobotics.ColorSensorV3.LEDCurrent;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.CatzColorConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.LocalADStarAK;
import frc.robot.subsystems.LEDs.SubsystemCatzLED;
import frc.robot.subsystems.LEDs.SubsystemCatzLED.LEDMode;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.vision.SubsystemCatzVision;
import frc.robot.CatzAutonomous;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  public static SubsystemCatzLED lead = SubsystemCatzLED.getInstance();
  private RobotContainer m_robotContainer;
  public static int flipDirection = 1;

  public static int latchedChecklistCounter = 0;
  
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
         Logger.recordMetadata
         ("GitDirty", "All changes committed");
         break;
      case 1:
         Logger.recordMetadata("GitDirty", "Uncomitted changes");
         break;
       default:
         Logger.recordMetadata("GitDirty", "Unknown");
         break;
   }

    // Set up data receivers & replay source
    switch (CatzConstants.currentMode) {
      // Running on a real robot, log to a USB stick
      case REAL:
        // Logger.addDataReceiver(new WPILOGWriter("/media/sda1/Logs/"));
        // Logger.addDataReceiver(new NT4Publisher  ());
        
        // new PowerDistribution(1, ModuleType.kRev);
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        // Logger.addDataReceiver(new WPILOGWriter("F:/robotics code projects/loggingfiles/"));
        // Logger.addDataReceiver(new NT4Publisher());
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    // Start AdvantageKit logger
    Logger.start();

    //instantiate the robot subsystems and commands using an object
    m_robotContainer = new RobotContainer();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); //YALL BETTER NOT DELETE THIS :D

    if(SubsystemCatzVision.getInstance().getAprilTagID(1) == 263 || SubsystemCatzVision.getInstance().getAprilTagID(0) == 263) { 
      lead.colorSolid(Color.kYellow); 
      lead.ledMode = LEDMode.Solid;
    }
  }

  @Override
  public void disabledInit() {}

  private AllianceColor prevAllianceColor = null;
  private boolean allianceColorChanged = false;
  private double allianceColorChangedTime = 0.0;
  private final double WAIT_UNTIL_RAINBOW_SEC = 3.0;

  @Override
  public void disabledPeriodic() {
    CatzAutonomous.getInstance().chooseAllianceColor();

    // if(prevAllianceColor != CatzAutonomous.getInstance().getAllianceColor()){
    //   allianceColorChanged = true;
    //   allianceColorChangedTime = Timer.getFPGATimestamp();
    // }

    if(CatzAutonomous.getInstance().getAllianceColor() == AllianceColor.Blue) { 
      lead.colorSolid(Color.kBlue); 
    }else{
      lead.colorSolid(Color.kRed); 
    }
    lead.ledMode = LEDMode.Solid;
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    CatzAutonomous.getInstance().chooseAllianceColor();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    SubsystemCatzShooter.getInstance().cmdSetKeepShooterOn(false).execute();
  }

  @Override
  public void teleopInit() {
    CatzAutonomous.getInstance().chooseAllianceColor();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if(CatzAutonomous.getInstance().getAllianceColor() == AllianceColor.Red){
      flipDirection = -1;
    }
  }

  @Override
  public void teleopPeriodic() {

    m_robotContainer.logDpadStates();
 
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

}
