package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.MtrConfigConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {

    //configured from front robot facing perspective
    private final TalonFX SHOOTER_MOTOR_RT;
    private final TalonFX SHOOTER_MOTOR_LT;

    private CANSparkMax LOAD_MOTOR;
    private CANSparkMax FEED_MOTOR;

    //tunable motor velocities
    LoggedTunableNumber shooterVelLT = new LoggedTunableNumber("LTVelShooter", 75);
    LoggedTunableNumber shooterVelRT = new LoggedTunableNumber("RTVelShooter", 90);
    LoggedTunableNumber feederMotor  = new LoggedTunableNumber("LoadMotor",  1);
    LoggedTunableNumber feederMotor2 = new LoggedTunableNumber("FeedMotor", 0.6);


    TalonFX[] shooterArray = new TalonFX[2];

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

                //create new config objects
    private TalonFXConfiguration      talonConfigs = new TalonFXConfiguration();
    private Slot0Configs         shooterMtrConfigs = new Slot0Configs();
    
    LoggedTunableNumber turretMotor  = new LoggedTunableNumber("TurretMotor",  0);


    private Slot1Configs turretMtrConfigs = new Slot1Configs();
    
    private double turretPositionDeg;


    
    public ShooterIOReal() {

        //shooterArray[2] = TURRET_MOTOR;
        talonConfigs.Slot1 = turretMtrConfigs;  //TBD


        //Drive Motor setup
        
        SHOOTER_MOTOR_RT = new TalonFX(10);
        SHOOTER_MOTOR_LT = new TalonFX(0);

        //neo
        LOAD_MOTOR = new CANSparkMax(3, MotorType.kBrushless);
        LOAD_MOTOR.restoreFactoryDefaults();
        LOAD_MOTOR.setSmartCurrentLimit(MtrConfigConstants.NEO_CURRENT_LIMIT_AMPS);
        LOAD_MOTOR.setIdleMode(IdleMode.kBrake);
        LOAD_MOTOR.enableVoltageCompensation(12.0);

        //neo 550
        FEED_MOTOR = new CANSparkMax(1, MotorType.kBrushless);
        FEED_MOTOR.restoreFactoryDefaults();
        FEED_MOTOR.setSmartCurrentLimit(MtrConfigConstants.NEO_CURRENT_LIMIT_AMPS);
        FEED_MOTOR.setIdleMode(IdleMode.kBrake);
        FEED_MOTOR.enableVoltageCompensation(12.0);

                //create shooter mtr array for easier calls
        shooterArray[0] = SHOOTER_MOTOR_RT;
        shooterArray[1] = SHOOTER_MOTOR_LT;

            //reset to factory defaults
        SHOOTER_MOTOR_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_LT.getConfigurator().apply(new TalonFXConfiguration());

    
        talonConfigs.Slot0 = shooterMtrConfigs;
            //current limit
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = MtrConfigConstants.FALCON_ENABLE_CURRENT_LIMIT;
        talonConfigs.CurrentLimits.SupplyCurrentLimit       = MtrConfigConstants.FALCON_CURRENT_LIMIT_AMPS;
        talonConfigs.CurrentLimits.SupplyCurrentThreshold   = MtrConfigConstants.FALCON_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigs.CurrentLimits.SupplyTimeThreshold      = MtrConfigConstants.FALCON_CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            //pid
        shooterMtrConfigs.kP = 0.11;
        shooterMtrConfigs.kI = 0.0;
        shooterMtrConfigs.kD = 0.0;
        shooterMtrConfigs.kV = 0.1189;


        //check if motors are initialized correctly
        for(int i=0;i<2;i++){
            initializationStatus = shooterArray[i].getConfigurator().apply(talonConfigs);
             if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID for shooter "+ shooterArray.toString());
        }

        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        inputs.shooterVelocityLT      = SHOOTER_MOTOR_LT.getVelocity().getValue();
        inputs.shooterVelocityRT      = SHOOTER_MOTOR_RT.getVelocity().getValue();
        inputs.shooterVelocityErrorLT = SHOOTER_MOTOR_LT.getClosedLoopError().getValue();
        inputs.shooterVelocityErrorRT = SHOOTER_MOTOR_RT.getClosedLoopError().getValue();
        inputs.shooterMotorVoltageLT  = SHOOTER_MOTOR_LT.getMotorVoltage().getValue();
        inputs.shooterMotorVoltageRT  = SHOOTER_MOTOR_RT.getMotorVoltage().getValue();
        inputs.shooterDutyCycleLT     = SHOOTER_MOTOR_LT.getDutyCycle().getValue();
        inputs.shooterDutyCycleRT     = SHOOTER_MOTOR_RT.getDutyCycle().getValue();
        inputs.shooterTorqueCurrentLT = SHOOTER_MOTOR_LT.getTorqueCurrent().getValue();
        inputs.shooterTorqueCurrentRT = SHOOTER_MOTOR_RT.getTorqueCurrent().getValue();

        inputs.LoadMotorPercentOutput = LOAD_MOTOR.get();
        inputs.LoadMotorVelocity = (LOAD_MOTOR.getEncoder().getVelocity()/60); //to rps

        inputs.FeedPercentOutput = FEED_MOTOR.get();
        inputs.FeedVelocity = (FEED_MOTOR.getEncoder().getVelocity()/60); //to rps

    }

    @Override
    public void shootWithVelocity() {
    
        double shooterVelocityLT = shooterVelLT.get();
        double shooterVelocityRT = shooterVelRT.get();

        SHOOTER_MOTOR_LT.setControl(new VelocityVoltage(-shooterVelocityLT));
        SHOOTER_MOTOR_RT.setControl(new VelocityVoltage(shooterVelocityRT));

        FEED_MOTOR.set(-feederMotor2.get());

    }



    @Override
    public void setShooterDisabled() {

        SHOOTER_MOTOR_LT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_RT.setControl(new DutyCycleOut(0));

        FEED_MOTOR.set(0);
    }


    @Override
    public void shootFeederWithVelocity() {
        LOAD_MOTOR.set(-feederMotor.get());
    }


    
    @Override
    public void shootFeederReverse() {

        LOAD_MOTOR.set(feederMotor.get());
    }

    
    
    @Override
    public void setFeederDisabled() {
        LOAD_MOTOR.set(0);
    }



    @Override
    public void setTurretPosition(double targetEncPos) {
    }
}
