package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbIOReal implements ClimbIO {

    public static int CLIMB_MOTOR_ID_LT = 40;
    public static int CLIMB_MOTOR_ID_RT = 41; 

    //Kraken configuration constants
    public static final int     KRAKEN_CURRENT_LIMIT_AMPS            = 55;
    public static final int     KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    public static final double  KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    public static final boolean KRAKEN_ENABLE_CURRENT_LIMIT          = true;

    private final TalonFX climbMtrLT;
    private final TalonFX climbMtrRT;

    private TalonFXConfiguration climbTalonConfigs = new TalonFXConfiguration();
    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;


    public ClimbIOReal() {
        climbMtrLT = new TalonFX(40); //TBD
        climbMtrRT = new TalonFX(41);

        climbMtrLT.getConfigurator().apply(new TalonFXConfiguration());
        climbMtrRT.getConfigurator().apply(new TalonFXConfiguration());

            //current limit
        climbTalonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        climbTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = KRAKEN_ENABLE_CURRENT_LIMIT;
        climbTalonConfigs.CurrentLimits.SupplyCurrentLimit       = KRAKEN_CURRENT_LIMIT_AMPS;
        climbTalonConfigs.CurrentLimits.SupplyCurrentThreshold   = KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS;
        climbTalonConfigs.CurrentLimits.SupplyTimeThreshold      = KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        climbTalonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //check if climb motor is initialized correctly
        initializationStatus = climbMtrRT.getConfigurator().apply(climbTalonConfigs);
        if(!initializationStatus.isOK()) {
            System.out.println("Failed to Configure Climb Mtr Controller CAN ID" + CLIMB_MOTOR_ID_LT);
        }
        initializationStatus = climbMtrLT.getConfigurator().apply(climbTalonConfigs);
        if(!initializationStatus.isOK()) {
            System.out.println("Failed to Configure Climb Mtr Controller CAN ID" + CLIMB_MOTOR_ID_LT);
        }

        climbMtrLT.setInverted(true);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        // inputs.climbDutyCycleLT = climbMtrLT.getDutyCycle().getValue();
        //inputs.climbPositionErrorLT = climbMtrLT.getClosedLoopError().getValue();
        // inputs.climbSpoolRevLT = climbMtrLT.getPosition().getValue();
        // inputs.climbTorqueCurrentLT = climbMtrLT.getTorqueCurrent().getValue();
        // inputs.climbVoltageLT = climbMtrLT.getMotorVoltage().getValue();

        // inputs.climbDutyCycleRT = climbMtrRT.getDutyCycle().getValue();
        // //inputs.climbPositionErrorRT = climbMtrRT.getClosedLoopError().getValue();
        // inputs.climbSpoolRevRT = climbMtrRT.getPosition().getValue();
        // inputs.climbTorqueCurrentRT = climbMtrRT.getTorqueCurrent().getValue();
        // inputs.climbVoltageRT = climbMtrRT.getMotorVoltage().getValue();
    }

    @Override
    public void setClimbPositionLT(double climbPosition) {
        climbMtrLT.setControl(new MotionMagicVoltage(climbPosition));
    }

    @Override
    public void setClimbMtrPercentOutputLT(double output) {
        climbMtrLT.set(output);
    }

    @Override
    public void setClimbSelectedSensorPositionLT(double readPosition) {
        climbMtrLT.setPosition(readPosition);
    }

    
    @Override
    public void setClimbPositionRT(double climbPosition) {
        climbMtrRT.setControl(new MotionMagicVoltage(climbPosition));
    }

    @Override
    public void setClimbMtrPercentOutputRT(double output) {
        climbMtrRT.set(output);
    }

    @Override
    public void setClimbSelectedSensorPositionRT(double readPosition) {
        climbMtrRT.setPosition(readPosition);
    }



    
}
