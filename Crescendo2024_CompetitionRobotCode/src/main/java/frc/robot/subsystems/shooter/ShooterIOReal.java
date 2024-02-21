package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzConstants.MtrConfigConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {

    //Shooter and Feed Motor 
    //Configured from front robot facing perspective
    private final TalonFX SHOOTER_MOTOR_RT;
    private final TalonFX SHOOTER_MOTOR_LT;

    private final int SHOOTER_MOTOR_LT_CAN_ID = 21;
    private final int SHOOTER_MOTOR_RT_CAN_ID = 20;

    private final CANSparkMax LOAD_MOTOR;

    private final int LOAD_MOTOR_CAN_ID = 5;

    private final DigitalInput BACK_BEAM_BREAK  = new DigitalInput(8);
    private final DigitalInput FRONT_BEAM_BREAK = new DigitalInput(9);

    private Servo shooterLeftServo;
    private Servo shooterRightServo;

    private final int SERVO_LEFT_PWM_ID = 1;
    private final int SERVO_RIGHT_PWM_ID = 2;


    //Tunable motor velocities
    LoggedTunableNumber shooterVelLT = new LoggedTunableNumber("LTVelShooter", 90);
    LoggedTunableNumber shooterVelRT = new LoggedTunableNumber("RTVelShooter", 70);

    //Load motor speeds 
    private final double LOAD_MOTOR_SHOOTING_SPEED = 1;
    private final double LOAD_MOTOR_LOADING_SPEED = 0.4;
    private final double LOAD_MOTOR_BACKWARD_SPEED = 0.07;
    private final double LOAD_MOTOR_FWD_ADJUST_SPEED = 0.07;


    TalonFX[] shooterArray = new TalonFX[2];

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

    //Create new Talong FX config objects
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs         pidConfigs   = new Slot0Configs();

    
    //Xbox controller to get what buttons were pressed
    public static CommandXboxController xboxDrv;
    
    public ShooterIOReal() {
        //Xbox controller setup
        xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT);

        //Servo setup
        shooterLeftServo = new Servo(SERVO_LEFT_PWM_ID);
        shooterRightServo = new Servo(SERVO_RIGHT_PWM_ID);
        
        //Falcon Shooter Motor setup
        SHOOTER_MOTOR_LT = new TalonFX(SHOOTER_MOTOR_LT_CAN_ID);
        SHOOTER_MOTOR_RT = new TalonFX(SHOOTER_MOTOR_RT_CAN_ID);

        
        //Neo Load motor config
        LOAD_MOTOR = new CANSparkMax(LOAD_MOTOR_CAN_ID, MotorType.kBrushless);
        LOAD_MOTOR.restoreFactoryDefaults();
        LOAD_MOTOR.setSmartCurrentLimit(MtrConfigConstants.NEO_CURRENT_LIMIT_AMPS);
        LOAD_MOTOR.setIdleMode(IdleMode.kCoast);
        LOAD_MOTOR.enableVoltageCompensation(12.0); //TBD is this the default value?
        
        //Create shooter mtr array for easier calls
        shooterArray[0] = SHOOTER_MOTOR_RT;
        shooterArray[1] = SHOOTER_MOTOR_LT;

        
        //Reset to factory defaults
        SHOOTER_MOTOR_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_LT.getConfigurator().apply(new TalonFXConfiguration());

        
        //Current limit
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


        //Initialize motors and check if motors are initialized correctly
        for(int i=0;i<2;i++) {
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
        inputs.shooterDutyCycleLT = SHOOTER_MOTOR_LT.getDutyCycle().getValue();
        inputs.shooterDutyCycleRT = SHOOTER_MOTOR_RT.getDutyCycle().getValue();
        inputs.shooterTorqueCurrentLT = SHOOTER_MOTOR_LT.getTorqueCurrent().getValue();
        inputs.shooterTorqueCurrentRT = SHOOTER_MOTOR_RT.getTorqueCurrent().getValue();

        inputs.shooterBackBeamBreakBroken =  !BACK_BEAM_BREAK.get();
        inputs.isShooterFrontBeamBreakBroken = !FRONT_BEAM_BREAK.get();

        inputs.LoadMotorPercentOutput = LOAD_MOTOR.get();
        inputs.LoadMotorVelocity      = (LOAD_MOTOR.getEncoder().getVelocity()/60); //to rps
    }

  //-------------------------------------------Flywheel Methods------------------------------------------

    @Override
    public void setShooterEnabled() {
        double shooterVelocityLT = shooterVelLT.get();
        double shooterVelocityRT = shooterVelRT.get();

        SHOOTER_MOTOR_LT.setControl(new VelocityVoltage(-shooterVelocityLT).withEnableFOC(true));
        SHOOTER_MOTOR_RT.setControl(new VelocityVoltage(shooterVelocityRT).withEnableFOC(true));
    }
    @Override
    public void setShooterDisabled() {
        SHOOTER_MOTOR_LT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_RT.setControl(new DutyCycleOut(0));
        loadDisabled();
    }

  //-------------------------------------------Load Methods------------------------------------------

    @Override
    public void loadForward() {
        LOAD_MOTOR.set(-LOAD_MOTOR_SHOOTING_SPEED);
    }
    //Code that will be tested for double beambreaks
    @Override
    public void fineAdjustFwd() {
            LOAD_MOTOR.set(-LOAD_MOTOR_FWD_ADJUST_SPEED);
    }

    @Override
    public void fineAdjustBck() {
        LOAD_MOTOR.set(LOAD_MOTOR_BACKWARD_SPEED);
    }

    @Override
    public void loadNote() {
            LOAD_MOTOR.set(-LOAD_MOTOR_LOADING_SPEED);
    }

    @Override
    public void loadDisabled() {
        LOAD_MOTOR.set(0);
    }
    @Override
    public void loadBackward() {
        LOAD_MOTOR.set(LOAD_MOTOR_BACKWARD_SPEED);
    }

  //--------------------------------------------Servo Methods----------------------------------------

  @Override
  public void setServoPower(double power) {
    shooterLeftServo.set(power);
    shooterRightServo.set(power);
  }
}
