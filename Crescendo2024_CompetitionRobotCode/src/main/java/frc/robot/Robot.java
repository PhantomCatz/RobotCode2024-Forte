// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.LocalADStarAK;
import frc.robot.Utils.LEDs.CatzRGB;
import frc.robot.Utils.LEDs.ColorMethod;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
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
         Logger.recordMetadata("GitDirty", "All changes committed");
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
        // Logger.addDataReceiver(new WPILOGWriter("/media/sda1")); //uncomment later this thing was cluttering the rio log
        Logger.addDataReceiver(new NT4Publisher());
        
       // new PowerDistribution(1, ModuleType.kRev);
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter("F:/robotics code projects/loggingfiles/"));
        Logger.addDataReceiver(new NT4Publisher());
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
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
        SubsystemCatzDrivetrain.getInstance().printAverageWheelMagEncValues();

  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static CatzRGB led = new CatzRGB();

  public enum mechMode
  {
    AutoMode(Color.kGreen),
    ManualHoldMode(Color.kCyan),
    ManualMode(Color.kRed);

    public Color color;
    mechMode(Color color){
      this.color = color;
    }
  }

  public enum gamePiece{
    Cube(Color.kPurple),
    Cone(Color.kYellow),
    None(Color.kGhostWhite);

    public Color color;
    gamePiece(Color color){
      this.color = color;
    }
  }

  public enum gameModeLED{
    Autobalancing(led.oneColorFill, Color.kGreen),
    InAutonomous(led.startFlowing, led.PHANTOM_SAPPHIRE, Color.kWhite),
    MatchEnd(led.startFlowingRainbow),
    EndgameWheelLock(led.oneColorFillAllianceColor), 
    TeleOp(led.doNothing);

    public ColorMethod method;
    public Color[] color;
    private gameModeLED(ColorMethod method, Color... color)
    {
      this.method = method;
      this.color = color;
    }
  }

  public static mechMode intakeControlMode = mechMode.AutoMode;
  public static mechMode elevatorControlMode = mechMode.AutoMode;
  public static mechMode armControlMode = mechMode.AutoMode;
  public static gameModeLED currentGameModeLED = gameModeLED.MatchEnd;
  public static gamePiece currentGamePiece = gamePiece.None;
}

