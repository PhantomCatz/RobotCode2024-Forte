package frc.robot.subsystems.drivetrain;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

public class ModuleIOSim implements ModuleIO
{
    
    private final FlywheelSim STEER_SIM;
    private final FlywheelSim DRIVE_SIM;

    DutyCycleEncoder magEnc;

    public boolean driveDirectionFlipped = false;
    public double  driveAppliedVolts;
    public double  steerAppliedVolts;


    public ModuleIOSim()
    {
        STEER_SIM = new FlywheelSim(DCMotor.getNEO(1), 150/7, 0.025);
        DRIVE_SIM = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.004);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        inputs.driveMtrVelocity = ((DRIVE_SIM.getAngularVelocityRPM()/60000)*100);
        inputs.driveMtrSensorPosition = -999;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.steerAppliedVolts = steerAppliedVolts;

    }

    @Override
    public void setDriveSimPwrIO(double mtrOutputPercent) 
    {
        double volts = mtrOutputPercent * 12;
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        DRIVE_SIM.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setSteerSimPwrIO(double mtrOutputPercent) 
    {
        double volts = mtrOutputPercent * 12;
        steerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        STEER_SIM.setInputVoltage(steerAppliedVolts);
    }



    
   

    
}
