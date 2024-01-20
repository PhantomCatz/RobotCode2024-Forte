package frc.robot.subsystems.SubsystemCatzShooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.MtrConfigConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {

    //configured from front robot facing perspective
    private final TalonFX SHOOTER_MOTOR_BTM_LT;
    private final TalonFX SHOOTER_MOTOR_BTM_RT;
    private final TalonFX SHOOTER_MOTOR_TOP_RT;
    private final TalonFX SHOOTER_MOTOR_TOP_LT;

    //tunable motor velocities
    LoggedTunableNumber shooterVelBtmLt = new LoggedTunableNumber("BtmLtShooter", 70);
    LoggedTunableNumber shooterVelBtmRt = new LoggedTunableNumber("BtmRtShooter", 50);
    LoggedTunableNumber shooterVelTopRt = new LoggedTunableNumber("TopRtShooter", 50);
    LoggedTunableNumber shooterVelTopLt = new LoggedTunableNumber("TopLtShooter", 70);

    TalonFX[] shooterArray = new TalonFX[4];

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

                //create new config objects
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs shooterMtrConfigs = new Slot0Configs();

    public ShooterIOReal() {
                //Drive Motor setup
        
        SHOOTER_MOTOR_BTM_RT = new TalonFX(0);
        SHOOTER_MOTOR_BTM_LT = new TalonFX(11); 
        SHOOTER_MOTOR_TOP_RT = new TalonFX(7);
        SHOOTER_MOTOR_TOP_LT = new TalonFX(31);

                //create shooter mtr array for easier calls
        shooterArray[0] = SHOOTER_MOTOR_BTM_RT;
        shooterArray[1] = SHOOTER_MOTOR_BTM_LT;
        shooterArray[2] = SHOOTER_MOTOR_TOP_RT;
        shooterArray[3] = SHOOTER_MOTOR_TOP_LT;

            //reset to factory defaults
        SHOOTER_MOTOR_BTM_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_BTM_LT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_TOP_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_TOP_LT.getConfigurator().apply(new TalonFXConfiguration());

    
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
        shooterMtrConfigs.kP = 0.2;
        shooterMtrConfigs.kI = 0.0;
        shooterMtrConfigs.kD = 0.0;


        //check if drive motor is initialized correctly
        for(int i=0;i<4;i++){
            initializationStatus = shooterArray[i].getConfigurator().apply(talonConfigs);
             if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID for shooter "+ shooterArray.toString());
        }

        SHOOTER_MOTOR_BTM_LT.setControl(new Follower(SHOOTER_MOTOR_TOP_LT.getDeviceID(), true));
        SHOOTER_MOTOR_BTM_RT.setControl(new Follower(SHOOTER_MOTOR_TOP_RT.getDeviceID(), true));

        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        //System.out.println("Cmd updateInputs");
        inputs.dummyVariable = 1;
        inputs.velocityBtmRT = SHOOTER_MOTOR_BTM_RT.getVelocity().getValue();
        inputs.velocityBtmLT = SHOOTER_MOTOR_BTM_LT.getVelocity().getValue();
        inputs.velocityTopRT = SHOOTER_MOTOR_TOP_RT.getVelocity().getValue();
        inputs.velocityTopLT = SHOOTER_MOTOR_TOP_LT.getVelocity().getValue();
    }

    @Override
    public void shootWithVelocity() {
        System.out.println("Cmd shootWithVelocity");
        //SHOOTER_MOTOR_BTM_RT.setControl(new VelocityVoltage(-shooterVelBtmRt.get()));
        //SHOOTER_MOTOR_BTM_LT.setControl(new VelocityVoltage(-shooterVelBtmLt.get()));
        SHOOTER_MOTOR_TOP_RT.setControl(new VelocityVoltage(-shooterVelTopRt.get()));
        SHOOTER_MOTOR_TOP_LT.setControl(new VelocityVoltage(shooterVelTopLt.get()));
    }

    @Override
    public void setShooterDisabled() {
        System.out.println("CMd off");
        //SHOOTER_MOTOR_BTM_LT.setControl(new DutyCycleOut(0));
        //SHOOTER_MOTOR_BTM_RT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_TOP_RT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_TOP_LT.setControl(new DutyCycleOut(0));
    }
}
