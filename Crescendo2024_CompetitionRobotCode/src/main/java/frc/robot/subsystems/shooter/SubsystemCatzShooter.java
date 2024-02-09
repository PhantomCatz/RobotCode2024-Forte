// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;


public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();


  //Flywheel and Feed constants and variables
  LoggedTunableNumber shooterVelRpsLT = new LoggedTunableNumber("LTVelShooter", 75.0);
  LoggedTunableNumber shooterVelRpsRT = new LoggedTunableNumber("RTVelShooter", 90.0);
  LoggedTunableNumber feedMotorTumableNumber = new LoggedTunableNumber("FeedMotor", 0.6);


  //Load constants and variables
  LoggedTunableNumber loadMotorTunableNumber  = new LoggedTunableNumber("LoadMotor",  1.0);

  //Pivot constants and variables
  private CatzMechanismPosition m_targetPosition;


  public SubsystemCatzShooter() {

    switch (CatzConstants.currentMode) {
      case REAL: io = new ShooterIOReal();
                 System.out.println("Shooter Configured for Real");
      break;

      case REPLAY: io = new ShooterIOReal() {};
                   System.out.println("Shooter Configured for Replayed Simulation");
      break;

      case SIM:
      default: io = null;
               System.out.println("Current Mode Unconfigured");
      break;
    }
  }


  // Get the singleton instance of the ShooterSubsystem
  public static SubsystemCatzShooter getInstance() {
      return instance;
  }



  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/shooterinputs ", inputs);

    if(DriverStation.isDisabled()) {
      io.setLoadDisabled();
      io.setShooterDisabled();
    } else {

      //TBD generate flag code for when teh shooters reach target vels
      //rumble?
      //if(inputs.shooterVelocityRpsLT > 1 && inputs.shooterVelcot)


      //TBD add in logic if the drivers want to turn off the shooter


      //TBD add logic for the beambreak /


      //TBD once we're done command shooters off

    }


    //logging variables
    SmartDashboard.putNumber("shooterRpsLT", inputs.shooterVelocityRpsLT);
    SmartDashboard.putNumber("shooterRpsRT", inputs.shooterVelocityRpsRT);

  }

  //-------------------------------------------Flywheel and Feed methods------------------------------------------

  public Command cmdShooterEnabled() {
    return run(()->shooterEnabled());
  }

  public Command cmdShooterDisabled() {
    return run(()->shooterDisabled());
  }

  public void shooterEnabled() {
    io.setFlywheelVelocity(shooterVelRpsLT.get(), shooterVelRpsRT.get());
    io.setFeedPercentOuput(feedMotorTumableNumber.get());
  }

  public void shooterDisabled() {
    io.setShooterDisabled();
    io.setFeedPercentOuput(0.0);
  }

  //--------------------------------------------------Load methods------------------------------------------
  public Command setLoadMotor() {
    return run(()->io.shootLoadPercentOutput(-loadMotorTunableNumber.get()));
  }

  public Command setLoadReverse() {
      return run(()->io.shootLoadPercentOutput(loadMotorTunableNumber.get()));
  }

  public Command setFeedMotorDisabled() {
    return run(()->io.setLoadDisabled());
  }

}
