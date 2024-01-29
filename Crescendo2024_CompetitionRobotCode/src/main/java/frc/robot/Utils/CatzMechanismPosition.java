// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;


public class CatzMechanismPosition {

    public double m_elevatorTargetEncPos;
    public double m_pivotTargetEncPos;

    public CatzMechanismPosition(double elevatorTargetEncPos, double pivotTargetEncPos) {
        this.m_elevatorTargetEncPos = elevatorTargetEncPos;
        this.m_pivotTargetEncPos = pivotTargetEncPos;
    }

    public double getElevatorTargetEncPos() {
        return m_elevatorTargetEncPos;
    }

    public double getPivotTargetEncPos() {
        return m_pivotTargetEncPos;
    }

}
