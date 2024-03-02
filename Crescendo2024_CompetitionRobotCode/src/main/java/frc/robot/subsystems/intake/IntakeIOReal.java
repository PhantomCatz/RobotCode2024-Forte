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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.MtrConfigConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class IntakeIOReal implements IntakeIO {

    public static int PIVOT_MTR_ID = 12;
    public static int ROLLER_MTR_ID = 10;
    
    private final DigitalInput intakeBeamBreak = new DigitalInput(4);

    private final TalonFX pivotMtr;
    private final TalonFX rollerMtr;

    private StatusCode pivotInitializationStatus  = StatusCode.StatusCodeNotInitialized;
    private StatusCode rollerInitializationStatus = StatusCode.StatusCodeNotInitialized;

            //create new config objects
    private TalonFXConfiguration talonConfigsPivot  = new TalonFXConfiguration();
    private TalonFXConfiguration talonConfigsRoller = new TalonFXConfiguration();


    public IntakeIOReal() {
        /************************************************************************************************************************
        * pivot
        ************************************************************************************************************************/
        pivotMtr = new TalonFX(PIVOT_MTR_ID);
        pivotMtr.getConfigurator().apply(new TalonFXConfiguration()); //reset to factory defaults

        // set Motion Magic settings
        talonConfigsPivot.MotionMagic.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
        talonConfigsPivot.MotionMagic.MotionMagicAcceleration   = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        talonConfigsPivot.MotionMagic.MotionMagicJerk           = 16000; // Target jerk of 1600 rps/s/s (0.1 seconds)


        talonConfigsPivot.Slot0.kP = 9.0;
        talonConfigsPivot.Slot0.kI = 0.00;
        talonConfigsPivot.Slot0.kD = 0.36;
        
            //current limit
        talonConfigsPivot.CurrentLimits = new CurrentLimitsConfigs();
        talonConfigsPivot.CurrentLimits.SupplyCurrentLimitEnable = MtrConfigConstants.FALCON_ENABLE_CURRENT_LIMIT; //TBD make these are apart of the the real class
        talonConfigsPivot.CurrentLimits.SupplyCurrentLimit       = MtrConfigConstants.FALCON_CURRENT_LIMIT_AMPS;  //TBD change to kraken
        talonConfigsPivot.CurrentLimits.SupplyCurrentThreshold   = MtrConfigConstants.FALCON_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigsPivot.CurrentLimits.SupplyTimeThreshold      = MtrConfigConstants.FALCON_CURRENT_LIMIT_TIMEOUT_SECONDS;

        talonConfigsPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        pivotMtr.setPosition(SubsystemCatzIntake.INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV);

        //check if wrist motor is initialized correctly
        pivotInitializationStatus = pivotMtr.getConfigurator().apply(talonConfigsPivot);
        if(!pivotInitializationStatus.isOK()) {
            System.out.println("Failed to Configure Pivot Mtr Controller CAN ID" + PIVOT_MTR_ID);
        }

        /************************************************************************************************************************
        * roller
        ************************************************************************************************************************/
        rollerMtr = new TalonFX(ROLLER_MTR_ID);
        rollerMtr.getConfigurator().apply(new TalonFXConfiguration());  //reset to factory defaults

        talonConfigsRoller = talonConfigsPivot;
        talonConfigsRoller.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        //check if roller motor is initialized correctly
        rollerInitializationStatus = rollerMtr.getConfigurator().apply(talonConfigsRoller);
        if(!rollerInitializationStatus.isOK()) {
            System.out.println("Failed to Configure Roller Mtr Controller CAN ID" + ROLLER_MTR_ID);
        }
    }


    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerVoltage =          rollerMtr.getMotorVoltage().getValue();
        inputs.pivotMtrRev =            pivotMtr.getPosition().getValue();
        inputs.rollerVoltage =          rollerMtr.getTorqueCurrent().getValue();
        inputs.pivotMtrPercentOutput =  pivotMtr.getDutyCycle().getValue();
        inputs.rollerPercentOutput =    rollerMtr.getDutyCycle().getValue();
        inputs.rollerVelocity =         rollerMtr.getVelocity().getValue();
        inputs.pivotMtrVelocityRPS =    pivotMtr.getVelocity().getValue();
        //true if beambreak is broken \/ \/
        inputs.isIntakeBeamBrkBroken =   !intakeBeamBreak.get(); //TBD add method for controling inputs
        inputs.closedLoopPivotMtr =     pivotMtr.getClosedLoopError().getValue();
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
    public void setIntakePivotVoltage(double volts) {
        pivotMtr.setControl(new VoltageOut(volts));
    }

    @Override
    public void setIntakePivotEncOutput(double encOutput, double ffVoltage) {
        pivotMtr.setControl(new MotionMagicVoltage(encOutput, 
                                                     true, 
                                                     ffVoltage, 
                                                     0, 
                                                     false, 
                                                     false, 
                                                     false));
    }


}

