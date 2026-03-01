// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import java.util.Optional;
import java.util.function.Supplier;
import yams.units.EasyCRT;
import yams.units.EasyCRT.CRTStatus;
import yams.units.EasyCRTConfig;

public class TurretIOKraken implements TurretIO {
  private final TalonFX motor;
  private final CANcoder cancoder10t;
  private final CANcoder cancoder17t;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  private final StatusSignal<Angle> motorPositionSignal;
  private final StatusSignal<AngularVelocity> motorVelocitySignal;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<Angle> cancoder10tPositionSignal;
  private final StatusSignal<Angle> cancoder17tPositionSignal;

  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer cancoder10tConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer cancoder17tConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final boolean crtResolveSucceeded;
  private final double crtResolvedAngleDeg;

  public TurretIOKraken() {
    motor = new TalonFX(TurretConstants.MOTOR_CAN_ID, TunerConstants.kCANBus);
    cancoder10t = new CANcoder(TurretConstants.CANCODER_10T_CAN_ID, TunerConstants.kCANBus);
    cancoder17t = new CANcoder(TurretConstants.CANCODER_17T_CAN_ID, TunerConstants.kCANBus);

    // Configure turret motor for mechanism-space position control and velocity-limited Motion
    // Magic.
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.MOTOR_TO_TURRET_RATIO;
    motorConfig.Slot0.kP = TurretConstants.kP;
    motorConfig.Slot0.kI = TurretConstants.kI;
    motorConfig.Slot0.kD = TurretConstants.kD;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        TurretConstants.MAX_VELOCITY_DEG_PER_SEC / 360.0;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        TurretConstants.MAX_ACCEL_DEG_PER_SEC_SQ / 360.0;

    if (TurretConstants.SOFT_LIMIT_ENABLED) {
      motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          TurretConstants.SOFT_LIMIT_MAX_DEG / 360.0;
      motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
          TurretConstants.SOFT_LIMIT_MIN_DEG / 360.0;
    }

    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));

    var cancoderConfig = new CANcoderConfiguration();
    tryUntilOk(5, () -> cancoder10t.getConfigurator().apply(cancoderConfig, 0.25));
    tryUntilOk(5, () -> cancoder17t.getConfigurator().apply(cancoderConfig, 0.25));

    motorPositionSignal = motor.getPosition();
    motorVelocitySignal = motor.getVelocity();
    motorAppliedVoltsSignal = motor.getMotorVoltage();
    motorCurrentSignal = motor.getStatorCurrent();
    cancoder10tPositionSignal = cancoder10t.getAbsolutePosition();
    cancoder17tPositionSignal = cancoder17t.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorPositionSignal,
        motorVelocitySignal,
        motorAppliedVoltsSignal,
        motorCurrentSignal,
        cancoder10tPositionSignal,
        cancoder17tPositionSignal);
    ParentDevice.optimizeBusUtilizationForAll(motor, cancoder10t, cancoder17t);

    // Fresh CANcoder reads for CRT (done once at construction)
    cancoder10t.getAbsolutePosition().refresh();
    cancoder17t.getAbsolutePosition().refresh();

    Supplier<Angle> enc10t = () -> cancoder10t.getAbsolutePosition().getValue();
    Supplier<Angle> enc17t = () -> cancoder17t.getAbsolutePosition().getValue();

    EasyCRTConfig crtConfig =
        new EasyCRTConfig(enc10t, enc17t)
            .withEncoderRatios(
                TurretConstants.CANCODER_10T_TO_TURRET, TurretConstants.CANCODER_17T_TO_TURRET)
            .withAbsoluteEncoderOffsets(
                Rotations.of(-TurretConstants.CANCODER_10T_OFFSET_ROT),
                Rotations.of(-TurretConstants.CANCODER_17T_OFFSET_ROT))
            .withMechanismRange(Rotations.of(-1.0), Rotations.of(1.0))
            .withMatchTolerance(Rotations.of(TurretConstants.CRT_MATCH_TOLERANCE_ROT))
            .withAbsoluteEncoderInversions(false, false);

    EasyCRT crt = new EasyCRT(crtConfig);
    Optional<Angle> resolved = crt.getAngleOptional();

    if (crt.getLastStatus() == CRTStatus.OK && resolved.isPresent()) {
      double angleDeg = resolved.get().in(Degrees);
      tryUntilOk(5, () -> motor.setPosition(angleDeg / 360.0));
      crtResolveSucceeded = true;
      crtResolvedAngleDeg = angleDeg;
    } else {
      DriverStation.reportError("[Turret] CRT resolve failed: " + crt.getLastStatus(), false);
      tryUntilOk(5, () -> motor.setPosition(0.0));
      crtResolveSucceeded = false;
      crtResolvedAngleDeg = 0.0;
    }
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            motorPositionSignal, motorVelocitySignal, motorAppliedVoltsSignal, motorCurrentSignal);
    var cancoder10tStatus = BaseStatusSignal.refreshAll(cancoder10tPositionSignal);
    var cancoder17tStatus = BaseStatusSignal.refreshAll(cancoder17tPositionSignal);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.cancoder10tConnected = cancoder10tConnectedDebounce.calculate(cancoder10tStatus.isOK());
    inputs.cancoder17tConnected = cancoder17tConnectedDebounce.calculate(cancoder17tStatus.isOK());
    inputs.turretPositionDeg = motorPositionSignal.getValueAsDouble() * 360.0;
    inputs.turretVelocityDegPerSec = motorVelocitySignal.getValueAsDouble() * 360.0;
    inputs.motorAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
    inputs.motorCurrentAmps = motorCurrentSignal.getValueAsDouble();
    inputs.cancoder10tPositionRot = cancoder10tPositionSignal.getValueAsDouble();
    inputs.cancoder17tPositionRot = cancoder17tPositionSignal.getValueAsDouble();
    inputs.crtResolveSucceeded = crtResolveSucceeded;
    inputs.crtResolvedAngleDeg = crtResolvedAngleDeg;
  }

  @Override
  public void setTargetAngleDeg(double angleDeg) {
    motor.setControl(motionMagicRequest.withPosition(angleDeg / 360.0));
  }
}
