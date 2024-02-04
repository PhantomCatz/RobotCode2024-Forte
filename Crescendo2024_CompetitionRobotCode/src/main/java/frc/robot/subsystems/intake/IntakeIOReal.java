package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.IntakeConstants;
import frc.robot.CatzConstants.MtrConfigConstants;

public class IntakeIOReal implements IntakeIO {

    private final DigitalInput beambreak = new DigitalInput(0); 

    private final TalonFX pivotMtr;
    private final TalonFX rollerMtr;

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;
    
            //create new config objects
    private TalonFXConfiguration pivotTalonConfigs = new TalonFXConfiguration();
    private Slot0Configs pivotConfigs = new Slot0Configs();
    private Slot1Configs rollerConfigs = new Slot1Configs();


    public IntakeIOReal() {
                //Wrist Motor setup
        pivotMtr = new TalonFX(IntakeConstants.PIVOT_MTR_ID); //TBD need mtr id
            //reset to factory defaults
        pivotMtr.getConfigurator().apply(new TalonFXConfiguration());
                //Wrist Motor setup
        rollerMtr = new TalonFX(IntakeConstants.PIVOT_MTR_ID); //TBD need mtr id
            //reset to factory defaults
        rollerMtr.getConfigurator().apply(new TalonFXConfiguration());
        pivotTalonConfigs.Slot0 = pivotConfigs;
        pivotTalonConfigs.Slot1 = rollerConfigs;
            //current limit
        pivotTalonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        pivotTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = MtrConfigConstants.FALCON_ENABLE_CURRENT_LIMIT;
        pivotTalonConfigs.CurrentLimits.SupplyCurrentLimit       = MtrConfigConstants.FALCON_CURRENT_LIMIT_AMPS;
        pivotTalonConfigs.CurrentLimits.SupplyCurrentThreshold   = MtrConfigConstants.FALCON_CURRENT_LIMIT_TRIGGER_AMPS;
        pivotTalonConfigs.CurrentLimits.SupplyTimeThreshold      = MtrConfigConstants.FALCON_CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        pivotTalonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            //pid
        pivotConfigs.kP = 0.1;
        pivotConfigs.kI = 0.0;
        pivotConfigs.kD = 0.001;

        rollerConfigs.kP = 0.1;
        rollerConfigs.kI = 0.0;
        rollerConfigs.kD = 0.001;

        //check if wrist motor is initialized correctly
        initializationStatus = pivotMtr.getConfigurator().apply(pivotTalonConfigs);
        if(!initializationStatus.isOK())
            System.out.println("Failed to Configure CAN ID" + IntakeConstants.PIVOT_MTR_ID);

        //check if roller motor is initialized correctly
        initializationStatus = rollerMtr.getConfigurator().apply(pivotTalonConfigs);
        initializationStatus = rollerMtr.getConfigurator().apply(pivotTalonConfigs.Slot1);
        if(!initializationStatus.isOK())
            System.out.println("Failed to Configure CAN ID" + IntakeConstants.ROLLER_MTR_ID);
    


    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // inputs.rollerVoltage = rollerMtr.getMotorVoltage().getValue();
    }

    @Override
    public void setPivotEncPos(double targetEncPos) {
        pivotMtr.setControl(new PositionVoltage(targetEncPos));
    }
    
    @Override
    public void resetPivotEncPos(double defaultEncPos) {
        pivotMtr.setPosition(defaultEncPos);
    }

    @Override
    public void setRollerPercentOutput(double speed) {
        rollerMtr.set(speed);
    }

    @Override
    public void setRollerVelocity(double velocity) {
        rollerMtr.setControl(new VelocityVoltage(velocity));
    }



}
