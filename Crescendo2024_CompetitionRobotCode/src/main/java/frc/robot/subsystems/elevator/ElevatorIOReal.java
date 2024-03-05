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
    public static int ELEVATOR_LT_MTR_ID = 50; //TBD ids are swapped for testing change back to 51
    public static int ELEVATOR_RT_MTR_ID = 51;

    //Kraken configuration constants
    public static final int     KRAKEN_CURRENT_LIMIT_AMPS            = 55;
    public static final int     KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    public static final double  KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    public static final boolean KRAKEN_ENABLE_CURRENT_LIMIT          = true;



                //create new config objects
    private TalonFXConfiguration elevatorTalonConfigs = new TalonFXConfiguration();
    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

    private DigitalInput m_bottomLimit = new DigitalInput(10);

    private final TalonFX ElevatorMtrRT;
    private final TalonFX ElevatorMtrLT;

    public ElevatorIOReal() {
        //Elevator Motor setup
        ElevatorMtrRT = new TalonFX(ELEVATOR_RT_MTR_ID);
        ElevatorMtrLT = new TalonFX(ELEVATOR_LT_MTR_ID);
            //reset to factory defaults
        ElevatorMtrLT.getConfigurator().apply(new TalonFXConfiguration());
        ElevatorMtrRT.getConfigurator().apply(new TalonFXConfiguration());

        // set Motion Magic settings
        elevatorTalonConfigs.MotionMagic.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
        elevatorTalonConfigs.MotionMagic.MotionMagicAcceleration   = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        elevatorTalonConfigs.MotionMagic.MotionMagicJerk           = 16000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        elevatorTalonConfigs.Slot0.kP = 1.5;
        elevatorTalonConfigs.Slot0.kI = 0.0;
        elevatorTalonConfigs.Slot0.kD = 0.0;
            //current limit
        elevatorTalonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        elevatorTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = KRAKEN_ENABLE_CURRENT_LIMIT;
        elevatorTalonConfigs.CurrentLimits.SupplyCurrentLimit       = KRAKEN_CURRENT_LIMIT_AMPS;
        elevatorTalonConfigs.CurrentLimits.SupplyCurrentThreshold   = KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS;
        elevatorTalonConfigs.CurrentLimits.SupplyTimeThreshold      = KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        elevatorTalonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        ElevatorMtrLT.setPosition(0);
        ElevatorMtrRT.setPosition(0);


        //check if elevator motor is initialized correctly
        ElevatorMtrLT.setControl(new Follower(ElevatorMtrRT.getDeviceID(), true));
        initializationStatus = ElevatorMtrRT.getConfigurator().apply(elevatorTalonConfigs);
        initializationStatus = ElevatorMtrLT.getConfigurator().apply(elevatorTalonConfigs);
        if(!initializationStatus.isOK()) {
            System.out.println("Failed to Configure Elevator Mtr Controller CAN ID" + ELEVATOR_RT_MTR_ID);
        }
            
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorVoltage          = ElevatorMtrRT.getMotorVoltage().getValue();
        inputs.elevatorDutyCycle        = ElevatorMtrRT.getDutyCycle().getValue();
        inputs.elevatorTorqueCurrent    = ElevatorMtrRT.getTorqueCurrent().getValue();
        inputs.elevatorVelocity         = ElevatorMtrRT.getVelocity().getValue();
        inputs.elevatorPosRev           = ElevatorMtrRT.getPosition().getValue();
        inputs.elevatorPositionError    = ElevatorMtrRT.getClosedLoopError().getValue();

        inputs.bottomSwitchTripped      = m_bottomLimit.get();
    }
    
    @Override
    public void setElevatorPosition(double newPositionElevator, double elevatorFF) {
        ElevatorMtrRT.setControl(new MotionMagicVoltage(newPositionElevator,
                                                        true, 
                                                        elevatorFF,
                                                        0, 
                                                        false,
                                                        false,
                                                        false));
    }

    @Override
    public void setElevatorVoltage(double volts) {
        ElevatorMtrRT.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElevatorPercentOutput(double speed) {
        ElevatorMtrRT.set(speed);
    }

    @Override
    public void setSelectedSensorPosition(double setNewReadPosition) {
        ElevatorMtrRT.setPosition(setNewReadPosition);
    }

}