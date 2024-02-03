package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs driveConfigs = new Slot0Configs();

    public IntakeIOReal() {
                //Wrist Motor setup
        pivotMtr = new TalonFX(IntakeConstants.PIVOT_MTR_ID); //TBD need mtr id
            //reset to factory defaults
        pivotMtr.getConfigurator().apply(new TalonFXConfiguration());
                //Wrist Motor setup
        rollerMtr = new TalonFX(IntakeConstants.PIVOT_MTR_ID); //TBD need mtr id
            //reset to factory defaults
        rollerMtr.getConfigurator().apply(new TalonFXConfiguration());
        talonConfigs.Slot0 = driveConfigs;
            //current limit
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = MtrConfigConstants.FALCON_ENABLE_CURRENT_LIMIT;
        talonConfigs.CurrentLimits.SupplyCurrentLimit       = MtrConfigConstants.FALCON_CURRENT_LIMIT_AMPS;
        talonConfigs.CurrentLimits.SupplyCurrentThreshold   = MtrConfigConstants.FALCON_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigs.CurrentLimits.SupplyTimeThreshold      = MtrConfigConstants.FALCON_CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            //pid
        driveConfigs.kP = 0.1;
        driveConfigs.kI = 0.0;
        driveConfigs.kD = 0.001;

        //check if wrist motor is initialized correctly
        initializationStatus = pivotMtr.getConfigurator().apply(talonConfigs);
        if(!initializationStatus.isOK())
            System.out.println("Failed to Configure CAN ID" + IntakeConstants.PIVOT_MTR_ID);

        //check if roller motor is initialized correctly
        initializationStatus = rollerMtr.getConfigurator().apply(talonConfigs);
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
