package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Robot;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.commands.mechanismCmds.AimAndOrFireAtSpeakerCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.IntakeManualCmd;
import frc.robot.commands.mechanismCmds.MoveToPresetHandoffCmd;
import frc.robot.commands.mechanismCmds.ScoreAmpCmd;
import frc.robot.subsystems.LEDs.LEDSection.LEDMode;
import frc.robot.subsystems.LEDs.SubsystemCatzLED;
import frc.robot.subsystems.climb.SubsystemCatzClimb;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.commands.mechanismCmds.MoveToPresetHandoffCmd;

public class CatzStateMachine extends SubsystemBase {

    private static NoteDestination targetNoteDestination = NoteDestination.SPEAKER;
    public static CatzStateMachine instance = new CatzStateMachine();

    public static CatzStateMachine getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("statemachine/note destination", targetNoteDestination);
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
