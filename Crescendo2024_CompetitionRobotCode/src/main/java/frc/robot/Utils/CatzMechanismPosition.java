// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;


public class CatzMechanismPosition {

    private double m_elevatorTargetEncPos;
    private double m_intakePivotTargetAngle;
    private double m_shooterTargetHorizontalAngle;
    private double m_rollerState;

    public CatzMechanismPosition(double elevatorTargetEncPos, double intakePivotTargetAngle, double shooterTargetHorizontalAngle, int rollerState) {
        this.m_elevatorTargetEncPos = elevatorTargetEncPos;
        this.m_intakePivotTargetAngle = intakePivotTargetAngle;
        this.m_shooterTargetHorizontalAngle = shooterTargetHorizontalAngle;
        this.m_rollerState = rollerState;
    }

    public double getElevatorTargetRev() {
        return m_elevatorTargetEncPos;
    }

    public double getIntakePivotTargetAngle() {
        return m_intakePivotTargetAngle;
    }

    public double getshooterTargetHorizontalAngle() {
        return m_shooterTargetHorizontalAngle;
    }

    public double getTargetRollerState() {
        return m_rollerState;
    }

}
