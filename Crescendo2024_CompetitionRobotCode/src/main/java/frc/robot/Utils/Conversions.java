package frc.robot.Utils;

import frc.robot.CatzConstants;

public class Conversions {
    private static final double circumference = CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE;
    private static final double gearRatio = CatzConstants.DriveConstants.SDS_L2_PLUS_GEAR_RATIO;

    public static double RPSToMPS(double rps){
        return rps * circumference / gearRatio;
    }

    public static double MPSToRPS(double velocity){
        return velocity / circumference * gearRatio;
    }
}