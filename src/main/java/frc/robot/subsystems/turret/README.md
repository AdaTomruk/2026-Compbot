# Turret Subsystem

## ⚙️ Overview
The Turret subsystem provides a rotating platform for the robot's scoring mechanism. It uses a high-performance Kraken X60 motor for precise positioning and two CANcoders on coprime pinions to resolve absolute position using the Chinese Remainder Theorem (CRT).

---

## 🔌 Hardware Mapping
| Component | Hardware Type | CAN ID / Port | CAN Bus | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Turret Motor** | Kraken X60 | `20` | `rio` | Main drive motor |
| **Pinion 1 (10t)** | CANcoder | `21` | `rio` | Absolute feedback |
| **Pinion 2 (17t)** | CANcoder | `22` | `rio` | Absolute feedback |

### Gear Train
- **Stage 1:** 12t → 46t
- **Stage 2:** 10t → 85t (Main Gear)
- **Overall Ratio:** 32.5833:1
- **CRT Coverage:** 720° unique range via 10t and 17t coprime pinions on the 85t gear.

---

## 🏗️ Architecture & AdvantageKit
This subsystem strictly follows the AdvantageKit 3-file IO pattern.
* **Interface:** [TurretIO.java](TurretIO.java)
* **Real Hardware:** [TurretIOKraken.java](TurretIOKraken.java)
* **Simulation:** [TurretIOSim.java](TurretIOSim.java)
* **Mechanism Notes:** Uses YAMS EasyCRT for absolute position resolution on boot.

---

## 🔄 State Machine & Flow
The turret initializes by resolving its absolute position via two CANcoders. If resolution fails, it can be re-triggered manually.

```mermaid
stateDiagram-v2
    [*] --> WaitingForSignals : Robot init
    WaitingForSignals --> ResolvingCRT : All CANcoder signals valid debounced
    ResolvingCRT --> Resolved : EasyCRT returns valid angle
    ResolvingCRT --> ResolveFailed : EasyCRT returns empty / error
    ResolveFailed --> ResolvingCRT : retryCRTResolveCommand triggered
    Resolved --> Running : TalonFX seeded
    Running --> Running : periodic position control
```

---

## 🎮 Command API (Public Methods)
These are the primary Command factories exposed to RobotContainer.java for button bindings:
* **getOrientationDeg():** Returns the current measured turret angle in degrees.
* **setAbsoluteOrientationCommand(angleDeg):** Moves the turret to a specific absolute angle.
* **setRelativeOrientationCommand(deltaDeg):** Offsets the current target by a specific amount.
* **retryCRTResolveCommand():** Re-triggers the CRT resolution process (useful if sensors were offline at boot).

---

## 🧪 Testing & Simulation Requirements
* **JUnit Tests:** Currently not implemented (See TODO).
* **Sim Behavior:** [TurretIOSim.java](TurretIOSim.java) uses `DCMotorSim` and `PIDController` with `SIM_kP/kI/kD` gains to emulate physics. Visualization is provided via `Mechanism2d` and `Pose3d` logs.

---

## Any Additional Notes
* **Constants TODOs:** Several constants in `Constants.java` require hardware tuning:
    * Tune `kP/kD` for real hardware (currently estimated).
    * Calibrate `ZERO_OFFSET_DEG`.
    * Set specific CANcoder offsets once mounted.
* **Alerts:** The subsystem logs alerts if CRT resolution fails on boot.

* **Interface:** [TurretIO.java](src/main/java/frc/robot/subsystems/turret/TurretIO.java)
* **Real Hardware:** [TurretIOKraken.java](src/main/java/frc/robot/subsystems/turret/TurretIOKraken.java)
* **Simulation:** [TurretIOSim.java](src/main/java/frc/robot/subsystems/turret/TurretIOSim.java)

### CRT Initialization Flow
The turret uses the **YAMS EasyCRT** library (v2026.2.23) to resolve its absolute angle at startup.
1.  **Dual Sensors:** Two CANcoders are mounted on coprime pinions (10-tooth and 17-tooth).
2.  **Resolution:** Because the pinions have different gear ratios to the turret, their combined absolute positions uniqueley identify the turret's position over a 720° (2 rotation) range.
3.  **Seeding:** The resolved angle is used to seed the Kraken's internal rotor position. All subsequent tracking is done via the motor's high-resolution integrated encoder.
4.  **Fallback:** If CRT fails (e.g., `AMBIGUOUS` or `NO_SOLUTION`), the subsystem defaults to $0^\circ$ and raises a dashboard alert.

---

## 🔄 State Machine & Flow

```mermaid
stateDiagram-v2
    [*] --> Initializing: Robot startup
    Initializing --> CRTResolveOK: EasyCRT Status == OK
    Initializing --> CRTResolveFailed: AMBIGUOUS or NO_SOLUTION
    CRTResolveOK --> PositionControl: Motor encoder seeded
    CRTResolveFailed --> PositionControl: Fallback to zero position\ncrtResolveSucceeded=false logged
    PositionControl --> PositionControl: setAbsoluteOrientation(deg)\nor setRelativeOrientation(deltaDeg)
```

---

## 🎮 Command API (Public Methods)
These are the primary Command factories exposed to `RobotContainer.java`:
* **setAbsoluteOrientationCommand(double angleDeg):** Rotates the turret to a specific angle in degrees relative to the robot's forward heading.
* **setRelativeOrientationCommand(double deltaDeg):** Adjusts the turret's target by a specific amount relative to its *current commanded* target.

---

## 🧪 Testing & Simulation Requirements
* **Simulation Behavior:** The `TurretIOSim` uses a `DCMotorSim` with a Kraken X60 FOC model and a WPILib `PIDController` to approximate the MotionMagic behavior of the real hardware.
* **Verification:** Use `./gradlew simulateJava` to verify turret motion in the AdvantageScope 3D visualizer.

---

## 📝 Additional Notes
* **Gear Ratios:**
    * Motor-to-Turret: $32.5833:1$ $((\frac{46}{12}) \times (\frac{85}{10}))$
    * 10t CANcoder: $8.5$ rotations per turret rotation.
    * 17t CANcoder: $5.0$ rotations per turret rotation.
* **Control Mode:** Uses `MotionMagicVoltage` for smooth, velocity-limited transitions.
* **Soft Limits:** Currently defined in `Constants.java` but disabled (`SOFT_LIMIT_ENABLED = false`) until mechanical ranges are finalized.
