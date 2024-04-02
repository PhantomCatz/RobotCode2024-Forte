package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.AccelStrategy;

public class TurretIOReal implements TurretIO {

    private final CANSparkMax turretMtr;

    public static final int NEO_CURRENT_LIMIT_AMPS = 30;

    public TurretIOReal() {
       
        turretMtr = new CANSparkMax(60, MotorType.kBrushless);
        turretMtr.restoreFactoryDefaults();
        turretMtr.setSmartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);
        turretMtr.setIdleMode(IdleMode.kBrake);
        turretMtr.enableVoltageCompensation(12.0);
        turretMtr.getEncoder().setPositionConversionFactor(1.0/SubsystemCatzTurret.TURRET_MOTOR_SHAFT_REV_PER_DEG);

        turretMtr.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 32767);

        turretMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        turretMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        turretMtr.setSoftLimit(SoftLimitDirection.kForward, 80);
        turretMtr.setSoftLimit(SoftLimitDirection.kReverse, -80);

        turretMtr.burnFlash(); //save configs so if pwr lost to be reapplied


    }
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretMtrPercentOutput = turretMtr.getAppliedOutput();
        // inputs.turretMtrOutputCurrent = turretMtr.getOutputCurrent();
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



}

