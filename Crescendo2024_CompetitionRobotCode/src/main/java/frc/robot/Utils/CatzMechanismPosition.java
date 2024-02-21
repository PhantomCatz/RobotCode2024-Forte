// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;


public class CatzMechanismPosition {

    public double m_elevatorTargetEncPos;
    public double m_intakePivotTargetAngle;
    public double m_shooterTargetHorizontalAngle;
    public double m_shooterTargetRPSPos;

    public CatzMechanismPosition(double elevatorTargetEncPos, double intakePivotTargetAngle, double shooterTargetHorizontalAngle, double shooterTargetRPSPos) {
        this.m_elevatorTargetEncPos = elevatorTargetEncPos;
        this.m_intakePivotTargetAngle = intakePivotTargetAngle;
        this.m_shooterTargetHorizontalAngle = shooterTargetHorizontalAngle;
        this.m_shooterTargetRPSPos = shooterTargetRPSPos;
    }

    public double getElevatorTargetEncPos() {
        return m_elevatorTargetEncPos;
    }

    public double getIntakePivotTargetEnc() {
        return m_intakePivotTargetAngle;
    }

    public double getShooterTargetRPSPos() {
        return m_shooterTargetRPSPos;
    }

    public double getshooterTargetHorizontalAngle() {
        return m_shooterTargetHorizontalAngle;
    }

}
