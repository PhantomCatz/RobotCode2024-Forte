// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;


public class CatzMechanismPosition {

    private double m_elevatorTargetEncPos;
    private double m_intakePivotTargetAngle;
    private double m_shooterVerticalTargetAngle;
    private double m_turretTargetAngle;

    public CatzMechanismPosition(double elevatorTargetEncPos, double intakePivotTargetAngle, double shooterTargetHorizontalAngle, double turretTargetAngle) {
        this.m_elevatorTargetEncPos = elevatorTargetEncPos;
        this.m_intakePivotTargetAngle = intakePivotTargetAngle;
        this.m_shooterVerticalTargetAngle = shooterTargetHorizontalAngle;
        this.m_turretTargetAngle = turretTargetAngle;
    }

    public double getElevatorTargetRev() {
        return m_elevatorTargetEncPos;
    }

    public double getIntakePivotTargetAngle() {
        return m_intakePivotTargetAngle;
    }

    public double getShooterVerticalTargetAngle() {
        return m_shooterVerticalTargetAngle;
    }

    public double getTurretTargetAngle() {
        return m_turretTargetAngle;
    }

}
