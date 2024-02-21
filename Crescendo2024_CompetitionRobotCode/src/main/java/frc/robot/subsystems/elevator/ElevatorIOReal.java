package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.CatzConstants.MtrConfigConstants;

public class ElevatorIOReal implements ElevatorIO {
                //create new config objects
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs elevatorConfigs = new Slot0Configs();
    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

    private DigitalInput m_forwardLimit = new DigitalInput(0);
    private DigitalInput m_reverseLimit = new DigitalInput(1);

    private final TalonFX ElevatorMtrRT;
    private final TalonFX ElevatorMtrLT;

    public ElevatorIOReal() {
        //Elevator Motor setup
        ElevatorMtrRT = new TalonFX(0);
        ElevatorMtrLT = new TalonFX(0);
            //reset to factory defaults
        ElevatorMtrRT.getConfigurator().apply(new TalonFXConfiguration());
        talonConfigs.Slot0 = elevatorConfigs;
            //current limit
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = MtrConfigConstants.FALCON_ENABLE_CURRENT_LIMIT;
        talonConfigs.CurrentLimits.SupplyCurrentLimit       = MtrConfigConstants.FALCON_CURRENT_LIMIT_AMPS;
        talonConfigs.CurrentLimits.SupplyCurrentThreshold   = MtrConfigConstants.FALCON_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigs.CurrentLimits.SupplyTimeThreshold      = MtrConfigConstants.FALCON_CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            //pid
        elevatorConfigs.kP = 2.5; //TBD 
        elevatorConfigs.kI = 0.0;
        elevatorConfigs.kD = 0.05;
            //ramping

        //check if elevator motor is initialized correctly
        initializationStatus = ElevatorMtrRT.getConfigurator().apply(talonConfigs);
        initializationStatus = ElevatorMtrLT.getConfigurator().apply(talonConfigs);
        if(!initializationStatus.isOK())
            System.out.println("Failed to Configure CAN ID" + 0);
            
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorVoltage = ElevatorMtrRT.getMotorVoltage().getValue();
        inputs.elevatorDutyCycle = ElevatorMtrRT.getDutyCycle().getValue();
        inputs.elevatorTorqueCurrent = ElevatorMtrRT.getTorqueCurrent().getValue();
        inputs.elevatorVelocity = ElevatorMtrRT.getVelocity().getValue();

        inputs.forwardSwitchTripped = m_forwardLimit.get();
        inputs.reverseSwitchTripped = m_reverseLimit.get();
    }
    
    @Override
    public void setElevatorPosition(double newPositionElevator) {
        ElevatorMtrRT.setControl(new PositionVoltage(newPositionElevator)
            .withLimitForwardMotion(true)
            .withLimitReverseMotion(true)
        );
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