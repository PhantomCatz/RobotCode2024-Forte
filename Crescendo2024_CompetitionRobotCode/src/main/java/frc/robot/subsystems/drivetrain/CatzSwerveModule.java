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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.Conversions;

public class CatzSwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private PIDController m_PID;

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.26);
                
    private final double kP = 0.25;
    private final double kI = 0.00;
    private final double kD = 0.000;

    private double m_wheelOffset;

    private int m_index;

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index, double angleOffset) {
        this.m_index = index;

        switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel);
                break;
            case SIM : io = 
                    null;
                break;
            default : io = 
                    new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel) {};
                break;
        }

        m_PID = new PIDController(kP, kI, kD);

        m_wheelOffset = offset;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module " + Integer.toString(m_index), inputs);

        //Logging outputs
        Logger.recordOutput("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());
        Logger.recordOutput("angle" + Integer.toString(m_index) , getCurrentRotation().getDegrees());
        //Logger.recordOutput("angletarget" + Integer.toString(m_index) , m_state.angle.getDegrees());


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

    private SwerveModuleState m_state;

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
        //ff drive control
        double driveRPSFF = m_driveFeedforward.calculate(driveRPS);
        //set drive velocity
        setDriveVelocity(driveRPS + driveRPSFF);

        //calculate steer pwr
        //negative steer power because of coordinate system
        double steerPIDpwr = - m_PID.calculate(currentAngleRad, targetAngleRad); 
        setSteerPower(steerPIDpwr);

        //logging
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
