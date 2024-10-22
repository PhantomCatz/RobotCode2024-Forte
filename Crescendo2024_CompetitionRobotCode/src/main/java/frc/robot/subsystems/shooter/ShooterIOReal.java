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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.LEDs.SubsystemCatzLED;

public class ShooterIOReal implements ShooterIO {
/*-----------------------------------------------------------------------------------------
 * 
 * Shooter Motors
 * Configured from front robot facing perspective
 * 
 *-----------------------------------------------------------------------------------------*/
    private TalonFX SHOOTER_MOTOR_RT;
    private TalonFX SHOOTER_MOTOR_LT;

    private final int SHOOTER_MOTOR_LT_CAN_ID = 21;
    private final int SHOOTER_MOTOR_RT_CAN_ID = 20;

    //Kraken configuration constants
    public static final int     KRAKEN_CURRENT_LIMIT_AMPS            = 55;
    public static final int     KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    public static final double  KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    public static final boolean KRAKEN_ENABLE_CURRENT_LIMIT          = true;
/*-----------------------------------------------------------------------------------------
 *
 * Load Motors
 * 
 *---------------------------------------------------------------------------------------*/
    private CANSparkMax LOAD_MOTOR;
    private final int LOAD_MOTOR_CAN_ID = 23;

    //Load motor speeds 
    private final double LOAD_MOTOR_SHOOTING_SPEED   = 1.0;
    private final double LOAD_MOTOR_LOADING_SPEED    = 0.6;//0.3;//0.6; //was 0.4 LISA
    private final double LOAD_MOTOR_BACKWARD_SPEED   = 0.8;
    private final double LOAD_MOTOR_ADJUST_SPEED     = 0.04;
    private final double LOAD_MOTOR_TRANSFER_SPEED   = 0.4;

    public static final int NEO_CURRENT_LIMIT_AMPS      = 30;

/*---------------------------------------------------------------------------------------
 * Beam Breaks
 *-------------------------------------------------------------------------------------*/
    private DigitalInput ADJUST_BEAM_BREAK = new DigitalInput(0);
    private DigitalInput LOAD_BEAM_BREAK   = new DigitalInput(1);

/*---------------------------------------------------------------------------------------
 * Linear Servos
 *-------------------------------------------------------------------------------------*/    
    private Servo shooterServoLT;
    private Servo shooterServoRT;

    private final int SERVO_LEFT_PWM_ID  = 0;
    private final int SERVO_RIGHT_PWM_ID = 1;

    private static final int SERVO_PW_US_MAX_POSITION          = 2000;
    private static final int SERVO_PW_US_MAX_DEADBAND_POSITION = 1800;
    private static final int SERVO_PW_US_CENTER_POSITION       = 1500;
    private static final int SERVO_PW_US_MIN_DEADBAND_POSITION = 1200;
    private static final int SERVO_PW_US_MIN_POSITION          = 1000;

/*---------------------------------------------------------------------------------------
 * Motor Velocities
 *-------------------------------------------------------------------------------------*/  

    public static int ACCEPTABLE_VEL_ERROR = 20;
    public double velocityThresholdLT;
    public double velocityThresholdRT;

    TalonFX[] shooterArray = new TalonFX[2];

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;
    
    //Create new Talong FX config objects
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs         pidConfigs   = new Slot0Configs();
    
    public ShooterIOReal() {
        //Servo setup
        shooterServoLT = new Servo(SERVO_LEFT_PWM_ID);
        shooterServoRT = new Servo(SERVO_RIGHT_PWM_ID);

        shooterServoLT.setBoundsMicroseconds(SERVO_PW_US_MAX_POSITION,    SERVO_PW_US_MAX_DEADBAND_POSITION, 
                                             SERVO_PW_US_CENTER_POSITION, SERVO_PW_US_MIN_DEADBAND_POSITION, 
                                             SERVO_PW_US_MIN_POSITION);


        shooterServoRT.setBoundsMicroseconds(SERVO_PW_US_MAX_POSITION,    SERVO_PW_US_MAX_DEADBAND_POSITION, 
                                             SERVO_PW_US_CENTER_POSITION, SERVO_PW_US_MIN_DEADBAND_POSITION, 
                                             SERVO_PW_US_MIN_POSITION);

        //Falcon Shooter Motor setup
        SHOOTER_MOTOR_LT = new TalonFX(SHOOTER_MOTOR_LT_CAN_ID);
        SHOOTER_MOTOR_RT = new TalonFX(SHOOTER_MOTOR_RT_CAN_ID);

        
        //Neo Load motor config
        LOAD_MOTOR = new CANSparkMax(LOAD_MOTOR_CAN_ID, MotorType.kBrushless);
        LOAD_MOTOR.restoreFactoryDefaults();
        LOAD_MOTOR.setSmartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);
        LOAD_MOTOR.setIdleMode(IdleMode.kBrake);
        LOAD_MOTOR.enableVoltageCompensation(12.0); 
        LOAD_MOTOR.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 32767);
        LOAD_MOTOR.burnFlash(); //save configs so if pwr lost to be reapplied


        
        //Create shooter mtr array for easier calls
        shooterArray[0] = SHOOTER_MOTOR_RT;
        shooterArray[1] = SHOOTER_MOTOR_LT;

        //Reset to factory defaults
        SHOOTER_MOTOR_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_LT.getConfigurator().apply(new TalonFXConfiguration());

        
        //Current limit
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();

        talonConfigs.CurrentLimits.StatorCurrentLimitEnable = KRAKEN_ENABLE_CURRENT_LIMIT;
        talonConfigs.CurrentLimits.StatorCurrentLimit       = KRAKEN_CURRENT_LIMIT_AMPS;

        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = KRAKEN_ENABLE_CURRENT_LIMIT; //Make seperate current limits
        talonConfigs.CurrentLimits.SupplyCurrentLimit       = KRAKEN_CURRENT_LIMIT_AMPS;
        talonConfigs.CurrentLimits.SupplyCurrentThreshold   = KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigs.CurrentLimits.SupplyTimeThreshold      = KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS;

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
                System.out.println("Failed to Configure CAN ID for shooter ");//+ shooterArray.toString()); // add in later
        }
    }


    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        inputs.shooterVelocityLT   =  SHOOTER_MOTOR_LT.getVelocity().getValue();
        inputs.shooterVelocityRT   =  SHOOTER_MOTOR_RT.getVelocity().getValue();

        // inputs.shooterDutyCycleLT       = SHOOTER_MOTOR_LT.getDutyCycle().getValue();
        // inputs.shooterDutyCycleRT       = SHOOTER_MOTOR_RT.getDutyCycle().getValue();
        // inputs.shooterTorqueCurrentLT   = SHOOTER_MOTOR_LT.getTorqueCurrent().getValue();
        // inputs.shooterTorqueCurrentRT   = SHOOTER_MOTOR_RT.getTorqueCurrent().getValue();

        inputs.shooterLoadBeamBreakState   = !LOAD_BEAM_BREAK.get();
        inputs.shooterAdjustBeamBreakState = !ADJUST_BEAM_BREAK.get();

        inputs.loadMotorEncCnts = LOAD_MOTOR.getEncoder().getPosition();

    }

  //-------------------------------------------Flywheel Methods------------------------------------------

    @Override
    public void setShooterEnabled(double shooterVelocityLT, double shooterVelocityRT) {
        SHOOTER_MOTOR_LT.setControl(new VelocityVoltage(-shooterVelocityLT).withEnableFOC(true));
        SHOOTER_MOTOR_RT.setControl(new VelocityVoltage( shooterVelocityRT).withEnableFOC(true));
    }

    @Override
    public void setShooterDisabled() {
        SHOOTER_MOTOR_LT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_RT.setControl(new DutyCycleOut(0));
        loadDisabled();
        SubsystemCatzLED.getInstance().isAligned = false;
    }

  //-------------------------------------------Load Methods------------------------------------------

    @Override
    public void feedShooter() {
        LOAD_MOTOR.set(-LOAD_MOTOR_SHOOTING_SPEED);
    }
    
    @Override
    public void fineAdjustFwd() {
        LOAD_MOTOR.set(-LOAD_MOTOR_ADJUST_SPEED);
    }

     @Override
    public void fineTransferAdjust() {
        LOAD_MOTOR.set(-LOAD_MOTOR_TRANSFER_SPEED);
    }

    @Override
    public void fineAdjustBck() {
        LOAD_MOTOR.set(LOAD_MOTOR_ADJUST_SPEED);
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
    public void resetLoadEnc() {
        LOAD_MOTOR.getEncoder().setPosition(0.0);
    }

    @Override
    public void loadBackward() {
        LOAD_MOTOR.set(LOAD_MOTOR_BACKWARD_SPEED);
    }

  //--------------------------------------------Servo Methods----------------------------------------

  @Override
  public void setServoPosition(double position) {
    shooterServoLT.set(position);
    shooterServoRT.set(position);
  }
  
  @Override
  public void setServoRetract() {
      shooterServoLT.set(0);
      shooterServoRT.set(0);
  }   
  
  @Override
  public void setServoAngle(double angle) {
    shooterServoLT.setAngle(angle);
    shooterServoRT.setAngle(angle);
  } 


}

