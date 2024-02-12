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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.MtrConfigConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.CatzConstants.ShooterConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {

    //Shooter and Feed Motor 
    //configured from front robot facing perspective
    private final TalonFX SHOOTER_MOTOR_RT;
    private final TalonFX SHOOTER_MOTOR_LT;
    private final CANSparkMax FEED_MOTOR;

    private final int SHOOTER_MOTOR_LT_CAN_ID = 0;
    private final int SHOOTER_MOTOR_RT_CAN_ID = 10;

    private final DigitalInput beamBreak = new DigitalInput(0);

    //tunable motor velocities
    LoggedTunableNumber shooterVelLT = new LoggedTunableNumber("LTVelShooter", 75); //was 75
    LoggedTunableNumber shooterVelRT = new LoggedTunableNumber("RTVelShooter", 90);
    LoggedTunableNumber feedMotor = new LoggedTunableNumber("FeedMotor", 0.61); //was 0.61
    private final double LOAD_MOTOR_SHOOTING_SPEED = 1;
    private final double LOAD_MOTOR_LOADING_SPEED = 0.15;

    TalonFX[] shooterArray = new TalonFX[2];

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

        //create new Talong FX config objects
    private TalonFXConfiguration      talonConfigs = new TalonFXConfiguration();
    private Slot0Configs         pidConfigs = new Slot0Configs();

    //Load Motor
    private final CANSparkMax LOAD_MOTOR;
    
    //Xbox controller to get what buttons were pressed
    public static CommandXboxController xboxDrv;
    
    public ShooterIOReal() {
        //xbox controller setup
        xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT);

        //Falcon Shooter Motor setup
        SHOOTER_MOTOR_LT = new TalonFX(SHOOTER_MOTOR_LT_CAN_ID); //TBD update this during testing in phoenix tunner
        SHOOTER_MOTOR_RT = new TalonFX(SHOOTER_MOTOR_RT_CAN_ID);

        //Neo Load motor config
        LOAD_MOTOR = new CANSparkMax(3, MotorType.kBrushless);
        LOAD_MOTOR.restoreFactoryDefaults();
        LOAD_MOTOR.setSmartCurrentLimit(MtrConfigConstants.NEO_CURRENT_LIMIT_AMPS);
        LOAD_MOTOR.setIdleMode(IdleMode.kCoast);
        LOAD_MOTOR.enableVoltageCompensation(12.0); //TBD is this the default value?

        //neo 550 Feed Motor config
        FEED_MOTOR = new CANSparkMax(1, MotorType.kBrushless);
        FEED_MOTOR.restoreFactoryDefaults();
        FEED_MOTOR.setSmartCurrentLimit(MtrConfigConstants.NEO_CURRENT_LIMIT_AMPS); //TBD make seperate current limit for 550
        FEED_MOTOR.setIdleMode(IdleMode.kCoast);
        FEED_MOTOR.enableVoltageCompensation(12.0);//TBD is this the default value?

            //create shooter mtr array for easier calls
        shooterArray[0] = SHOOTER_MOTOR_RT;
        shooterArray[1] = SHOOTER_MOTOR_LT;

            //reset to factory defaults
        SHOOTER_MOTOR_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_LT.getConfigurator().apply(new TalonFXConfiguration());

        //current limit
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = MtrConfigConstants.FALCON_ENABLE_CURRENT_LIMIT; //Make seperate current limits
        talonConfigs.CurrentLimits.SupplyCurrentLimit       = MtrConfigConstants.FALCON_CURRENT_LIMIT_AMPS;
        talonConfigs.CurrentLimits.SupplyCurrentThreshold   = MtrConfigConstants.FALCON_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigs.CurrentLimits.SupplyTimeThreshold      = MtrConfigConstants.FALCON_CURRENT_LIMIT_TIMEOUT_SECONDS;

        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        //pid
        talonConfigs.Slot0 = pidConfigs;
        pidConfigs.kP = 0.11; //TBD
        pidConfigs.kI = 0.0;
        pidConfigs.kD = 0.0;
        pidConfigs.kV = 0.1189; //TBD 


        //initialize motors and check if motors are initialized correctly
        for(int i=0;i<2;i++){
            initializationStatus = shooterArray[i].getConfigurator().apply(talonConfigs);
             if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID for shooter "+ shooterArray.toString());
        }

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        inputs.shooterVelocityLT   = SHOOTER_MOTOR_LT.getVelocity().getValue();
        inputs.shooterVelocityRT   = SHOOTER_MOTOR_RT.getVelocity().getValue();
        inputs.shooterVelocityErrorLT = SHOOTER_MOTOR_LT.getClosedLoopError().getValue();
        inputs.shooterVelocityErrorRT = SHOOTER_MOTOR_RT.getClosedLoopError().getValue();
        inputs.shooterMotorVoltageLT  = SHOOTER_MOTOR_LT.getMotorVoltage().getValue();
        inputs.shooterMotorVoltageRT  = SHOOTER_MOTOR_RT.getMotorVoltage().getValue();
        inputs.shooterPercentOutputLT = SHOOTER_MOTOR_LT.getDutyCycle().getValue();
        inputs.shooterPercentOutputRT = SHOOTER_MOTOR_RT.getDutyCycle().getValue();
        inputs.shooterTorqueCurrentLT = SHOOTER_MOTOR_LT.getTorqueCurrent().getValue();
        inputs.shooterTorqueCurrentRT = SHOOTER_MOTOR_RT.getTorqueCurrent().getValue();

        inputs.LoadMotorPercentOutput = LOAD_MOTOR.get();
        inputs.LoadMotorVelocity      = (LOAD_MOTOR.getEncoder().getVelocity()/60); //to rps

        inputs.FeedPercentOutput      = FEED_MOTOR.get();
        inputs.FeedVelocity           = (FEED_MOTOR.getEncoder().getVelocity()/60); //to rps

        inputs.BBShooterUnbroken = beamBreak.get();
    }

    @Override
    public void setShooterEnabled() {
        double shooterVelocityLT = shooterVelLT.get();
        double shooterVelocityRT = shooterVelRT.get();

        SHOOTER_MOTOR_LT.setControl(new VelocityVoltage(-shooterVelocityLT));
        SHOOTER_MOTOR_RT.setControl(new VelocityVoltage(shooterVelocityRT));
        /*Commands.waitSeconds(2);
        SHOOTER_MOTOR_LT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_RT.setControl(new DutyCycleOut(0));
        feedDisabled();
        loadDisabled();*/

    }

    @Override
    public void setShooterDisabled() {
        SHOOTER_MOTOR_LT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_RT.setControl(new DutyCycleOut(0));
        feedDisabled();
        loadDisabled();
    }

    
    @Override
    public void loadForward() {
        if(xboxDrv.x().getAsBoolean() == true) {
            LOAD_MOTOR.set(-LOAD_MOTOR_SHOOTING_SPEED);
        } else {
            if (beamBreak.get() == true) {
                LOAD_MOTOR.set(-LOAD_MOTOR_LOADING_SPEED);
            } else {
                LOAD_MOTOR.set(0);
            }
        }
    }

    @Override
    public void loadReverse() {
        LOAD_MOTOR.set(LOAD_MOTOR_LOADING_SPEED);
    }
    
    @Override
    public void loadDisabled() {
        LOAD_MOTOR.set(0);
    }

    @Override
    public void feedForward() {
        FEED_MOTOR.set(-feedMotor.get());
        //loadForward();
    }
    
    @Override
    public void feedDisabled() {
        FEED_MOTOR.set(0);
    }

    /*@Override
    public void loadForwardCmd(boolean bbUnBroken) {
        if (bbUnBroken){
            loadForward();
        } else {
            loadDisabled();
        }
    }*/

    /*@Override
    public void setShooterEnabledCmd(boolean rdyToShoot) {
        setShooterEnabled();
        if (rdyToShoot){
            feedForward();
        } else {
            feedDisabled();
        }
    }*/

    @Override
    public void setTurretPosition(double targetEncPos) {}
}
