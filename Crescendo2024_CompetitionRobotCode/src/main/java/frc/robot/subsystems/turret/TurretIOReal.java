package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.CatzConstants.MtrConfigConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class TurretIOReal implements TurretIO {

    LoggedTunableNumber rollerMotorTunableNumber = new LoggedTunableNumber("IntakeRoller", 0.6);
    
   // private final DigitalInput beamBreakBack = new DigitalInput(4);
    //private final DigitalInput beamBreakFront = new DigitalInput(5);

    private final CANSparkMax turretMtr;

    public TurretIOReal() {
       
        turretMtr = new CANSparkMax(60, MotorType.kBrushless);
        turretMtr.restoreFactoryDefaults();
        turretMtr.setSmartCurrentLimit(MtrConfigConstants.NEO_CURRENT_LIMIT_AMPS);
        turretMtr.setIdleMode(IdleMode.kBrake);
        turretMtr.enableVoltageCompensation(12.0);
    }
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretMtrPercentOutput = turretMtr.get();
        inputs.turretMtrOutputCurrent = turretMtr.getOutputCurrent();
        inputs.turretEncValue         = turretMtr.getEncoder().getPosition();
        Logger.recordOutput("turretEncValue", inputs.turretEncValue);
    }

    @Override
    public void turretSetPwr(double outputPwr) {
        
        turretMtr.set(outputPwr);
    }

    @Override 
    public void turretSetEncoderPos(double position){
        turretMtr.getEncoder().setPosition(position);
    }

    @Override
    public double getTurretEncoderPos() {
        double turretEncPosition = turretMtr.getEncoder().getPosition();
        return turretEncPosition;
    }
}

