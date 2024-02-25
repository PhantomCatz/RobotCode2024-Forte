/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.Conversions;

public class CatzSwerveModule {
    //Module delcaration block
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    //object variable declaration
    private PIDController m_PID;
                
    //steering pid constants
    private final double kP = 0.6; 
    private final double kI = 0.01;
    private final double kD = 0.000;

    //global swerve module constants
    private double m_wheelOffset;
    private int m_index;

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index, double angleOffset) {
        this.m_index = index;

        switch (CatzConstants.currentMode) {
            case REAL: io = new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel);
                        System.out.println("Module " + driveMotorID + " Configured for Real");
            break;

            case REPLAY : io = new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel) {};
                        System.out.println("Module " + driveMotorID + " Configured for Replay simulation");
            break;

            case SIM:    
            default : io = null;
                        System.out.println("Module " + driveMotorID + " Unconfigured");
            break;
        }

        m_PID = new PIDController(kP, kI, kD);

        m_wheelOffset = offset;
        this.DRIVE_MOTOR_ID = driveMotorID;

        resetDriveEncs();

    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module " + Integer.toString(m_index), inputs);

        //Logging outputs
        Logger.recordOutput("Module/absenctorad" + Integer.toString(m_index) , getAbsEncRadians());
        Logger.recordOutput("Module/angle" + Integer.toString(m_index) , getCurrentRotation().getDegrees());
        Logger.recordOutput("Module " + m_index + "/input voltage", Math.abs(inputs.driveAppliedVolts));
        Logger.recordOutput("Module " + m_index + "/velocity", Math.abs(inputs.driveMtrVelocity));
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/drive velocity error", inputs.driveVelocityError);
        Logger.recordOutput("Module " + m_index + "/steer voltage", inputs.steerAppliedVolts);

        SmartDashboard.putNumber("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());
        SmartDashboard.putNumber("angle" + Integer.toString(m_index) , getCurrentRotation().getDegrees());
    }

    //----------------------------------------Setting pwr methods-------------------------------
    public void setSteerPower(double pwr) {
        io.setSteerPwrIO(pwr);
    }

    public void setDriveVelocity(double velocity) {
        io.setDriveVelocityIO(velocity);
    }

    public double getAverageRawMagEnc(){
        double sum = 0;

        for(int i = 0; i < 100; i++){
            sum += inputs.magEncoderValue;
            Timer.delay(0.01);
        }

        return sum/100.0;
    }

    public void stopDriving() {
        io.setDrivePwrPercentIO(0.0);
    }

    //----------------------------------Util Methods catzswerve------------------------
    public double getDrvDistanceRaw() {
        return inputs.driveMtrSensorPosition;
    }

    public void setCoastMode() {
        io.setSteerCoastModeIO();
    }

    public void setBrakeMode() {
        io.setSteerBrakeModeIO();
    }

    public double getDrvVelocity() {
        return inputs.driveMtrVelocity;
    }
    
    private double getAbsEncRadians() {
        return (inputs.magEncoderValue - m_wheelOffset) * 2 * Math.PI;
    }
    
    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        double unadjustedSpeedSetpoint = state.speedMetersPerSecond;
        double targetAngleRad          = state.angle.getRadians();
        double currentAngleRad         = getAbsEncRadians();
        // Run closed loop drive control

        //calculate drive pwr
        double driveRPS = Conversions.MPSToRPS(unadjustedSpeedSetpoint);

        //set drive velocity
        setDriveVelocity(driveRPS);

        //calculate steer pwr
        //negative steer power because of coordinate system
        double steerPIDpwr = -m_PID.calculate(currentAngleRad, targetAngleRad); 

        setSteerPower(steerPIDpwr);

        //logging
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/angle error deg", Math.toDegrees(targetAngleRad-currentAngleRad));
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/target state", state);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/current state", getModuleState());
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/turn power", steerPIDpwr);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/currentmoduleangle rad", currentAngleRad);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/targetmoduleangle rad", targetAngleRad);
    }

    //optimze wheel angles before sending to setdesiredstate method for logging
    public SwerveModuleState optimizeWheelAngles(SwerveModuleState unoptimizedState) {
        SwerveModuleState optimizedState = CatzMathUtils.optimize(unoptimizedState, getCurrentRotation()); 
        return optimizedState;
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
    }

    //inputs the rotation object as radian conversion
    public Rotation2d getCurrentRotation() {
        return new Rotation2d(getAbsEncRadians());
    }

    public SwerveModuleState getModuleState() {
        double velocityMPS = Conversions.RPSToMPS(inputs.driveMtrVelocity);
        
        return new SwerveModuleState(velocityMPS, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters() {
        // seconds cancels out
        return Conversions.RPSToMPS(inputs.driveMtrSensorPosition);
    }
}
