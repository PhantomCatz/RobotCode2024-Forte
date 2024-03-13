package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.mechanismCmds.AimAndOrFireAtSpeakerCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.IntakeManualCmd;
import frc.robot.commands.mechanismCmds.MoveToHandoffPoseCmd;
import frc.robot.commands.mechanismCmds.ScoreAmpCmd;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class CatzStateMachine extends SubsystemBase {
    
    private NoteDestination targetNoteDestination = NoteDestination.SPEAKER;

    private static CatzMechanismPosition previousPose = CatzMechanismConstants.POS_STOW;
    
    private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
    private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
    private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
    private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();


    public static CatzStateMachine instance = new CatzStateMachine();
    public static CatzStateMachine getInstance() {
        return instance;
    }


    @Override
    public void periodic() {
        Logger.recordOutput("statemachine/note destination", targetNoteDestination);

    }

    //-----------------------------------------------
    // setter methods
    //----------------------------------------------
    public Command cmdNewNoteDestintation(NoteDestination newDestination) {
        return runOnce(()->setNewNoteDestination(newDestination));
    }

    private void setNewNoteDestination(NoteDestination newDestination) {
        targetNoteDestination = newDestination;
    }

    public static void setPreviousPose(CatzMechanismPosition pose) {
        previousPose = pose;
    }


    //-----------------------------------------------
    // getter methods
    //----------------------------------------------
    public CatzMechanismPosition getPreviousPose() {
     return previousPose;   
    }


    public NoteDestination getNoteDestination() {
        return targetNoteDestination;
    }
    public enum NoteDestination {
        SPEAKER,
        AMP,
        TRAP,
        HOARD
    }
    public enum NoteSource {
        INTAKE_SOURCE,
        INTAKE_GROUND,
        FROM_SHOOTER,
        FROM_INTAKE,
        NULL
      }
}
