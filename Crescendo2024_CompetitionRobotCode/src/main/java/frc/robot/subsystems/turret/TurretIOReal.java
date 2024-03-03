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
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import frc.robot.CatzConstants.MtrConfigConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class TurretIOReal implements TurretIO {

    private final CANSparkMax turretMtr;
    private final SparkPIDController smartMotionPID;

    public TurretIOReal() {
       
        turretMtr = new CANSparkMax(60, MotorType.kBrushless);
        turretMtr.restoreFactoryDefaults();
        turretMtr.setSmartCurrentLimit(MtrConfigConstants.NEO_CURRENT_LIMIT_AMPS);
        turretMtr.setIdleMode(IdleMode.kBrake);
        turretMtr.enableVoltageCompensation(12.0);

        smartMotionPID = turretMtr.getPIDController();
        smartMotionPID.setP(0.02);
        smartMotionPID.setI(0);
        smartMotionPID.setD(0);

        smartMotionPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        smartMotionPID.setSmartMotionAllowedClosedLoopError(5, 0);
        smartMotionPID.setSmartMotionMaxVelocity(2000, 0);
        smartMotionPID.setSmartMotionMinOutputVelocity(0,0);
    }
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretMtrPercentOutput = turretMtr.get();
        inputs.turretMtrOutputCurrent = turretMtr.getOutputCurrent();
        inputs.turretEncValue         = turretMtr.getEncoder().getPosition();
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
    public void turretSetPosition(double position) {
        smartMotionPID.setReference(position, ControlType.kSmartMotion);
    }


}

