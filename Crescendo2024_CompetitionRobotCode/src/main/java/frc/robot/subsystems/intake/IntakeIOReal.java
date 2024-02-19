package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.IntakeConstants;
import frc.robot.CatzConstants.MtrConfigConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class IntakeIOReal implements IntakeIO {

    LoggedTunableNumber rollerMotorTunableNumber = new LoggedTunableNumber("IntakeRoller", 0.6);
    
    private final DigitalInput IntakeBeamBreak = new DigitalInput(4);
    //private final DigitalInput beamBreakFront = new DigitalInput(5);

    private final TalonFX pivotMtr;
    private final TalonFX rollerMtr;

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;
    
            //create new config objects
    private TalonFXConfiguration pivotTalonConfigs = new TalonFXConfiguration();
    private Slot0Configs pidConfigs = new Slot0Configs();
    private Slot1Configs rollerConfigs = new Slot1Configs();

    public IntakeIOReal() {
                //Wrist Motor setup
       pivotMtr = new TalonFX(IntakeConstants.PIVOT_MTR_ID);
            //reset to factory defaults
       pivotMtr.getConfigurator().apply(new TalonFXConfiguration());
                //Wrist Motor setup
        rollerMtr = new TalonFX(IntakeConstants.ROLLER_MTR_ID);
            //reset to factory defaults
        rollerMtr.getConfigurator().apply(new TalonFXConfiguration());
        pivotTalonConfigs.Slot0 = pidConfigs;
        pivotTalonConfigs.Slot1 = rollerConfigs;
            //current limit
        pivotTalonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        pivotTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = MtrConfigConstants.FALCON_ENABLE_CURRENT_LIMIT;
        pivotTalonConfigs.CurrentLimits.SupplyCurrentLimit       = MtrConfigConstants.FALCON_CURRENT_LIMIT_AMPS;
        pivotTalonConfigs.CurrentLimits.SupplyCurrentThreshold   = MtrConfigConstants.FALCON_CURRENT_LIMIT_TRIGGER_AMPS;
        pivotTalonConfigs.CurrentLimits.SupplyTimeThreshold      = MtrConfigConstants.FALCON_CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        pivotTalonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        pidConfigs.kP = 0.11;
        pidConfigs.kI = 0.0;
        pidConfigs.kD = 0.0;
        pidConfigs.kV = 0.1189;

        pivotMtr.setPosition(140);

        //check if wrist motor is initialized correctly
        initializationStatus = pivotMtr.getConfigurator().apply(pivotTalonConfigs);
            if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + IntakeConstants.PIVOT_MTR_ID);
        //check if roller motor is initialized correctly
        for(int i=0;i<1;i++) {
        initializationStatus = rollerMtr.getConfigurator().apply(pivotTalonConfigs);
        if(!initializationStatus.isOK())
            System.out.println("Failed to Configure CAN ID" + IntakeConstants.ROLLER_MTR_ID);
        }
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerVoltage =          rollerMtr.getMotorVoltage().getValue();
        inputs.pivotMtrEncRev =         pivotMtr.getPosition().getValue();
        inputs.rollerVoltage =          rollerMtr.getTorqueCurrent().getValue();
        inputs.pivotMtrPercentOutput =  pivotMtr.getDutyCycle().getValue();
        inputs.rollerPercentOutput =    rollerMtr.getDutyCycle().getValue();
        inputs.rollerVelocity =         rollerMtr.getVelocity().getValue();
        //true if beambreak is broken \/ \/
        inputs.IntakeBeamBrkBroken = !IntakeBeamBreak.get(); //TBD add method for controling inputs
        //inputs.BeamBrkFrontBroken = !beamBreakFront.get();
        inputs.closedLoopPivotMtr = pivotMtr.getClosedLoopError().getValue();
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
    public void setIntakePivotPercentOutput(double percentOutput) {
       pivotMtr.set(-percentOutput);
    }

    @Override
    public void setIntakePivotEncOutput(double encOutput, double ffPercentOutput) {
        pivotMtr.setControl(new MotionMagicDutyCycle(encOutput, true, ffPercentOutput, 0, false, false, false));
    }


}

