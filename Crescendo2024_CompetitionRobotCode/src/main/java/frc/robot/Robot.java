// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.LocalADStarAK;
import frc.robot.Utils.LEDs.CatzRGB;
import frc.robot.Utils.LEDs.ColorMethod;
import frc.robot.subsystems.CatzStateMachine;
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

        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/Logs/"));
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
    // SubsystemCatzVision.getInstance().setUseSingleTag(true, 4);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); //YALL BETTER NOT DELETE THIS :D
    Logger.recordOutput("statemachine/note destination", CatzStateMachine.getInstance().getTargetNoteDestination());
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
  public void autonomousExit() {
    if(CatzAutonomous.chosenAllianceColor.get() == CatzConstants.AllianceColor.Red) {
      SubsystemCatzDrivetrain.getInstance().flipGyro();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

 
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
        // SubsystemCatzDrivetrain.getInstance().printAverageWheelMagEncValues();

  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  //-----------------------------------------------------------LED config------------------------------------------------
  public static CatzRGB led = new CatzRGB();

  //leds for mechanism state
  public enum mechMode {
    AutoMode(Color.kGreen),
    ManualHoldMode(Color.kCyan),
    ManualMode(Color.kRed);

    public Color color;
    mechMode(Color color){
      this.color = color;
    }
  }

  //leds for autoaligning
  public enum AutoAlignState {
    Aligned(Color.kGreen),
    Misaligned_Veritcal(Color.kPurple),
    Misaligned_Horizontal(Color.kYellow),
    TargetNotFound(Color.kBlack);

    public Color color;
    AutoAlignState(Color color){
      this.color = color;
    }
  }

  //leds for the type of robot note state mode we are in
  public enum manipulatorMode {
    Amp(Color.kOrange), Amp_No_Note(Color.kWhite),
    Speaker(Color.kOrange), Speaker_No_Note(Color.kWhite),
    Climb(Color.kBlue), Climb_No_Note(Color.kYellow),
    Hoard(Color.kAntiqueWhite),
    Source(Color.kRed),
    None(Color.kGhostWhite);

    public Color color;
    manipulatorMode(Color color){
      this.color = color;
    }
  }

    //default leds
  public enum gameModeLED{
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
  public static manipulatorMode currentGamePiece = manipulatorMode.None;
}

