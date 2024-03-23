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

import frc.robot.Utils.LoggedTunableNumber;

public class TurretIOReal implements TurretIO {

    private final CANSparkMax turretMtr;
    private final SparkPIDController smartMotionPID;

    public static final int     NEO_CURRENT_LIMIT_AMPS      = 30;

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

        smartMotionPID = turretMtr.getPIDController();
        smartMotionPID.setP(0.01);
        smartMotionPID.setI(0.0);
        smartMotionPID.setD(0.0);

        smartMotionPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        smartMotionPID.setSmartMotionAllowedClosedLoopError(3.0, 0);
        smartMotionPID.setSmartMotionMaxVelocity(2, 0);
        smartMotionPID.setSmartMotionMaxAccel(5.0, 0);
        smartMotionPID.setSmartMotionMinOutputVelocity(0,0);
        smartMotionPID.setOutputRange(-1, 1);

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

    @Override
    public void turretSetPositionSM(double reference) {
        smartMotionPID.setReference(reference, ControlType.kSmartMotion);
    }


}

