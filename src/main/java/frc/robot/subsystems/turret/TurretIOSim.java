// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim turretSim;
  private final PIDController controller =
      new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);

  private double targetAngleDeg = 0.0;
  private double appliedVolts = 0.0;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                GEARBOX,
                TurretConstants.TURRET_INERTIA_KG_M2,
                TurretConstants.MOTOR_TO_TURRET_RATIO),
            GEARBOX);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double targetMotorRad =
        targetAngleDeg / 180.0 * Math.PI * TurretConstants.MOTOR_TO_TURRET_RATIO;

    appliedVolts =
        MathUtil.clamp(
            controller.calculate(turretSim.getAngularPositionRad(), targetMotorRad), -12.0, 12.0);
    turretSim.setInputVoltage(appliedVolts);
    turretSim.update(0.02);

    double turretPositionDeg =
        turretSim.getAngularPositionRad()
            / TurretConstants.MOTOR_TO_TURRET_RATIO
            * (180.0 / Math.PI);
    double turretVelocityDegPerSec =
        turretSim.getAngularVelocityRadPerSec()
            / TurretConstants.MOTOR_TO_TURRET_RATIO
            * (180.0 / Math.PI);
    double turretPositionRot = turretPositionDeg / 360.0;

    inputs.motorConnected = true;
    inputs.cancoder10tConnected = true;
    inputs.cancoder17tConnected = true;
    inputs.turretPositionDeg = turretPositionDeg;
    inputs.turretVelocityDegPerSec = turretVelocityDegPerSec;
    inputs.motorAppliedVolts = appliedVolts;
    inputs.motorCurrentAmps = Math.abs(turretSim.getCurrentDrawAmps());
    inputs.cancoder10tPositionRot = turretPositionRot * TurretConstants.CANCODER_10T_TO_TURRET;
    inputs.cancoder17tPositionRot = turretPositionRot * TurretConstants.CANCODER_17T_TO_TURRET;
    inputs.crtResolveSucceeded = true;
    inputs.crtResolvedAngleDeg = turretPositionDeg;
  }

  @Override
  public void setTargetAngleDeg(double angleDeg) {
    targetAngleDeg = angleDeg;
  }
}
