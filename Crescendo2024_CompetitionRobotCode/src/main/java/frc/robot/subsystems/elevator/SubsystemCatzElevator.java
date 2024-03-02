// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.elevator;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.CatzConstants;
// import frc.robot.CatzConstants.CatzMechanismConstants;
// import frc.robot.CatzConstants.ElevatorConstants;
// import frc.robot.Utils.CatzMechanismPosition;
// import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
// import frc.robot.subsystems.intake.IntakeIOReal;
// import frc.robot.subsystems.intake.SubsystemCatzIntake;

// public class SubsystemCatzElevator extends SubsystemBase {
//   //instance instantiation
//   private static SubsystemCatzElevator instance = new SubsystemCatzElevator();

//   //io block
//   private final ElevatorIO io;
//   private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

//   //elevator constants
//   private static final double ElEVATOR_REV_TO_INCHES = 0.0;
//   private static final double ELEVATOR_GEAR_RATIO    = 0.0;

//   private static final double ELEVATOR_NULL_POSITION = -999.0;

//   private static final double ELEVATOR_MANUAL_STEP_SIZE = 0.5;

//   //elevator variables
//   private double m_newPositionRev;
//   private double m_elevatorPercentOutput;

//   private static ElevatorState currentElevatorState;
//   private static enum ElevatorState {
//     AUTO,
//     FULL_MANUAL,
//     SEMI_MANUAL
//   }

//   public SubsystemCatzElevator() {
//     switch (CatzConstants.currentMode) {
//       case REAL: io = new ElevatorIOReal();
//                  System.out.println("Elevator Configured for Real");
//       break;

//       case REPLAY: io = new ElevatorIOReal() {};
//                    System.out.println("Elevator Configured for Replayed simulation");
//       break;

//       case SIM:
//       default: io = null;
//                System.out.println("Elevator Unconfigured");
//       break;
//     }
//   }

//   // Get the singleton instance of the elevator Subsystem
//   public static SubsystemCatzElevator getInstance() {
//       return instance;
//   }

//   @Override
//   public void periodic() {
//     io.updateInputs(inputs);
//     Logger.processInputs("Elevator/inputs", inputs);
//     if(DriverStation.isDisabled() && 
//        SubsystemCatzIntake.getInstance().getWristAngle() > 60) {
//       io.setElevatorPercentOutput(0);
//     }
//     else {
//       if((m_newPositionRev != ELEVATOR_NULL_POSITION) && 
//           currentElevatorState == ElevatorState.AUTO  &&
//           currentElevatorState == ElevatorState.SEMI_MANUAL) {
//         io.setElevatorPosition(m_newPositionRev);
//           }
//       else {
//         io.setElevatorPercentOutput(m_elevatorPercentOutput);
//       }
//     }
//   }

//   public void updateElevatorTargetRev(double targetPos) {
//     currentElevatorState = ElevatorState.AUTO;
//     m_newPositionRev = targetPos;
//   }

//   public void setElevatorSemiManualPwr(double output) {
//     currentElevatorState = ElevatorState.SEMI_MANUAL;
//     m_newPositionRev = inputs.elevatorPosRev * (output * ELEVATOR_MANUAL_STEP_SIZE);
//   }

//   public void setElevatorPercentOutput(double percentOutput) {
//     currentElevatorState = ElevatorState.FULL_MANUAL;
//     this.m_elevatorPercentOutput = percentOutput/10;
//   }
//   // ----------------------------------------------------------------------------------
//   // Elevator getters
//   // ----------------------------------------------------------------------------------

//   public double getElevatorRevPos() {
//     return inputs.elevatorPosRev;
//   }

// }