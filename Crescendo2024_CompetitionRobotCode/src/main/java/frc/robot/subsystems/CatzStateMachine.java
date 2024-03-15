package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LEDs.LEDSection.LEDMode;
import frc.robot.commands.mechanismCmds.AimAndOrFireAtSpeakerCmd;
import frc.robot.commands.mechanismCmds.ManualElevatorCmd;
import frc.robot.commands.mechanismCmds.IntakeManualCmd;
import frc.robot.commands.mechanismCmds.MoveToHandoffPoseCmd;
import frc.robot.commands.mechanismCmds.ScoreAmpOrTrapCmd;
import frc.robot.subsystems.climb.SubsystemCatzClimb;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class CatzStateMachine extends SubsystemBase {
    
    private static NoteDestination targetNoteDestination = NoteDestination.SPEAKER;

    private static CatzMechanismPosition previousPose = CatzMechanismConstants.STOW;
    
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

        if(SubsystemCatzClimb.getInstance().isClimbing()){
            if(SubsystemCatzIntake.getInstance().getIntakeBeamBreakBroken()){
                Robot.lead.top.setModeNColor(Color.kBlue, Color.kBlue, LEDMode.Solid);
                Robot.lead.bot.setModeNColor(Color.kBlue, Color.kBlue, LEDMode.Solid);
                Robot.lead.mid.colorRainbow();
            }else{
                Robot.lead.top.setModeNColor(Color.kWhite, Color.kBlue, LEDMode.Alternating);
                Robot.lead.bot.setModeNColor(Color.kBlue, Color.kWhite, LEDMode.Alternating);
                Robot.lead.mid.colorRainbow();
            }
        }
        else if(targetNoteDestination == NoteDestination.AMP){
            if(SubsystemCatzIntake.getInstance().getIntakeBeamBreakBroken()){
                Robot.lead.top.setModeNColor(Color.kYellow, Color.kYellow, LEDMode.Solid);
                Robot.lead.bot.setModeNColor(Color.kYellow, Color.kYellow, LEDMode.Solid);
            }else{
                Robot.lead.top.setModeNColor(Color.kWhite, Color.kYellow, LEDMode.Alternating);
                Robot.lead.bot.setModeNColor(Color.kYellow, Color.kWhite, LEDMode.Alternating);
            }
        }
        else if(targetNoteDestination == NoteDestination.SPEAKER){
            if(SubsystemCatzShooter.getInstance().shooterLoadBeamBrkBroken()){
                Robot.lead.top.setModeNColor(Color.kOrange, Color.kOrange, LEDMode.Solid);
                Robot.lead.bot.setModeNColor(Color.kOrange, Color.kOrange, LEDMode.Solid);
            }else{
                Robot.lead.top.setModeNColor(Color.kOrange, Color.kWhite, LEDMode.Alternating);
                Robot.lead.bot.setModeNColor(Color.kWhite, Color.kOrange, LEDMode.Alternating);
            }
        }
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
