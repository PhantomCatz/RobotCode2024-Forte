package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.CatzConstants;

public class ElevatorIOReal implements ElevatorIO {

    //elevator motor ids
    public static int ELEVATOR_RT_MTR_ID = 50;

    //Kraken configuration constants
    public static final int     KRAKEN_CURRENT_LIMIT_AMPS            = 55;
    public static final int     KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    public static final double  KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    public static final boolean KRAKEN_ENABLE_CURRENT_LIMIT          = true;

                //create new config objects
    private TalonFXConfiguration elevatorTalonConfigs = new TalonFXConfiguration();
    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

    private DigitalInput m_bottomLimit = new DigitalInput(2);

    private final TalonFX ElevatorMtr;

    public ElevatorIOReal() {
        //Elevator Motor setup
        ElevatorMtr = new TalonFX(ELEVATOR_RT_MTR_ID);
        ElevatorMtr.setInverted(true);
            //reset to factory defaults
        ElevatorMtr.getConfigurator().apply(new TalonFXConfiguration());

        // set Motion Magic settings
        elevatorTalonConfigs.MotionMagic.MotionMagicCruiseVelocity = 260; // Target cruise velocity of 80 rps
        elevatorTalonConfigs.MotionMagic.MotionMagicAcceleration   = 400; // Target acceleration of 160 rps/s (0.5 seconds)
        elevatorTalonConfigs.MotionMagic.MotionMagicJerk           = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        elevatorTalonConfigs.Slot0.kP = 7.0;
        elevatorTalonConfigs.Slot0.kI = 0.0;
        elevatorTalonConfigs.Slot0.kD = 0.02;
            //current limit
        elevatorTalonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        elevatorTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = KRAKEN_ENABLE_CURRENT_LIMIT;
        elevatorTalonConfigs.CurrentLimits.SupplyCurrentLimit       = KRAKEN_CURRENT_LIMIT_AMPS;
        elevatorTalonConfigs.CurrentLimits.SupplyCurrentThreshold   = KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS;
        elevatorTalonConfigs.CurrentLimits.SupplyTimeThreshold      = KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        elevatorTalonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //ElevatorMtr.optimizeBusUtilization();

        ElevatorMtr.setPosition(0);


        //check if elevator motor is initialized correctly
        initializationStatus = ElevatorMtr.getConfigurator().apply(elevatorTalonConfigs);
        if(!initializationStatus.isOK()) {
            System.out.println("Failed to Configure Elevator Mtr Controller CAN ID" + ELEVATOR_RT_MTR_ID);
        }
            
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // inputs.elevatorVoltage          = ElevatorMtr.getMotorVoltage().getValue();
        // inputs.elevatorDutyCycle        = ElevatorMtr.getDutyCycle().getValue();
        // inputs.elevatorTorqueCurrent    = ElevatorMtr.getTorqueCurrent().getValue();
        // inputs.elevatorVelocity         = ElevatorMtr.getVelocity().getValue();
        inputs.elevatorPosRev           = ElevatorMtr.getPosition().getValue();
        inputs.elevatorPositionError    = ElevatorMtr.getClosedLoopError().getValue();

        inputs.bottomSwitchTripped      = m_bottomLimit.get();
    }
    
    @Override
    public void setElevatorPosition(double newPositionElevatorRev, double elevatorFF, boolean limitSwtichPressed) {
        ElevatorMtr.setControl(new MotionMagicVoltage(newPositionElevatorRev,
                                                        true, 
                                                        elevatorFF,
                                                        0, 
                                                        false,
                                                        false,
                                                        limitSwtichPressed));
    }

    @Override
    public void setElevatorVoltage(double volts) {
        ElevatorMtr.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElevatorPercentOutput(double speed) {
        ElevatorMtr.set(speed);
    }

    @Override
    public void setSelectedSensorPosition(double setNewReadPosition) {
        ElevatorMtr.setPosition(setNewReadPosition);
    }

    public double getElevatorError(){
        return ElevatorMtr.getClosedLoopError().getValue();
    }
}