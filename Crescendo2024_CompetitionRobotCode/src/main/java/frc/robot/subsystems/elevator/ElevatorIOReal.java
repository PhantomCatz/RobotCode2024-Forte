package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.CatzConstants.MtrConfigConstants;

public class ElevatorIOReal implements ElevatorIO {

                //create new config objects
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs elevatorConfigs = new Slot0Configs();
    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;


    private final TalonFX ElevatorMtr;

    public ElevatorIOReal() {

        //Drive Motor setup
        ElevatorMtr = new TalonFX(0);
            //reset to factory defaults
        ElevatorMtr.getConfigurator().apply(new TalonFXConfiguration());
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

        //check if drive motor is initialized correctly

            initializationStatus = ElevatorMtr.getConfigurator().apply(talonConfigs);
            if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + 0);
            
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.dummyVariable = 1;
        inputs.elevatorVoltage = ElevatorMtr.getMotorVoltage().getValue();
    }
    
    @Override
    public void setElevatorPosition(double newPositionElevator) {
        ElevatorMtr.setControl(new PositionVoltage(newPositionElevator));
    }

    @Override
    public void setElevatorPercentOutput(double speed) {
        ElevatorMtr.set(speed);
    }
}
