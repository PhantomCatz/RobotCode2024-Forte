// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;


public class CatzMechanismPosition {

    public double m_elevatorTargetEncPos;
    public double m_intakePivotTargetEncPos;
    public double m_shooterTargetRPS;

    public CatzMechanismPosition(double elevatorTargetEncPos, double intakePivotTargetEncPos, double shooterTargetRPS) {
        this.m_elevatorTargetEncPos = elevatorTargetEncPos;
        this.m_intakePivotTargetEncPos = intakePivotTargetEncPos;
        this.m_shooterTargetRPS = shooterTargetRPS;
    }

    public double getElevatorTargetEncPos() {
        return m_elevatorTargetEncPos;
    }

    public double getIntakePivotTargetEncPos() {
        return m_intakePivotTargetEncPos;
    }

    public double getShooterTargetRPSPos() {
        return m_intakePivotTargetEncPos;
    }

}
