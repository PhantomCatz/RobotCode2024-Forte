package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        
    }

    //-----------------------------------------------
    // setter methods
    //----------------------------------------------
    public Command cmdNewNoteDestintation(NoteDestination newDestination) {
        return run(()->setNewNoteDestination(newDestination));
    }

    private void setNewNoteDestination(NoteDestination newDestination) {
        targetNoteDestination = newDestination;
    }

    public Command cmdDetermineHandOffTransition() {
        if(targetNoteDestination == NoteDestination.AMP ||
           targetNoteDestination == NoteDestination.TRAP) {
            System.out.println("transfer to shooter");
            return new MoveToHandoffPoseCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER);
        } else {
            System.out.println("transfer to shooter");
            return new MoveToHandoffPoseCmd(NoteDestination.SPEAKER, NoteSource.FROM_INTAKE);

        }
    }

    public Command cmdDetermineButtonBCommand(Supplier<Boolean> supplierB) {
        switch(targetNoteDestination) {
            case SPEAKER:
            case HOARD:
                System.out.println("Speaker B cmd");
                return new AimAndOrFireAtSpeakerCmd(()->supplierB.get());
            case TRAP:
                System.out.println("Trap B cmd");
                return new SequentialCommandGroup();
            default:
                System.out.println("Amp B cmd");
                return new ScoreAmpCmd();
        }
    }

    public Command cmdDetermineAuxLeftSick(Supplier<Double> supplierLeftY, Supplier<Boolean> supplierLeftStickPressed) {
        switch(targetNoteDestination) {
            case SPEAKER:
            case HOARD:
                System.out.println("spkr ltst cmd");
                return shooter.cmdServoPosition(supplierLeftY);
            case AMP:
            case TRAP:
            default:
                System.out.println("Amp ltst cmd");
                return new ManualElevatorCmd(supplierLeftY, supplierLeftStickPressed);
        }
    }
    public Command cmdDetermineAuxRightSick(Supplier<Double> supplierRightY, Supplier<Boolean> supplierRightStickPressed) {
        switch(targetNoteDestination) {
            case SPEAKER:
            case HOARD:
                System.out.println("spkr rtst cmd");
                return shooter.cmdShooterRamp();
            case AMP:
            case TRAP:
            default:
                System.out.println("Amp rtst cmd");
                return new IntakeManualCmd(supplierRightY, supplierRightStickPressed);
        }
    }

    //-----------------------------------------------
    // getter methods
    //----------------------------------------------
    public NoteDestination getTargetNoteDestination() {
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
