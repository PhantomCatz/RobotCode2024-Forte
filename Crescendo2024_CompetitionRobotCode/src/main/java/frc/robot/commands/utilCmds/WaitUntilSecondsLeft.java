package frc.robot.commands.utilCmds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitUntilSecondsLeft extends Command{
    private double seconds;

    /**
     * This command should only run in Practice Mode
     */
    public WaitUntilSecondsLeft(double seconds){
        this.seconds = seconds;
    }

    @Override
    public boolean isFinished(){
        return DriverStation.getMatchTime() <= seconds;   
    }
    
}
