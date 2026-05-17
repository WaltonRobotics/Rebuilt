# /shooter

The fun part. This folder contains everything responsible for launching game pieces: the flywheel, the adjustable hood, and a full-range turret. Together they handle spinning up to speed, aiming at the correct angle, tracking a target across the field, and knowing when a ball has actually left the robot (without a sensor in the ball path, yes, really).

All of the actual shot math lives in [`calc/`](calc/), which runs on a background thread so it doesn't steal time from the robot loop. The classes here are purely responsible for controlling real hardware.

---

## Files

- `Shooter.java`: Master subsystem. Owns the flywheel motors, holds references to Hood and Turret, handles ball detection, and applies all shot parameters from the calc thread.
- `Turret.java`: Controls the turret's rotation. Homes itself using two absolute encoders with a fancy math algorithm (more on that below).
- `Hood.java`: Controls the hood's pitch angle. Homes by driving into a hard stop and watching the current spike.
- `FuelSim.java`: *(Commented out)* Physics simulation of ball trajectories for visualization in sim mode. Still here if we want it back.
- `TurretVisualizer.java`: *(Commented out)* 3D turret visualization for AdvantageScope.

---

## Shooter.java

The main subsystem. Two Kraken X44s spin a flywheel, with Motor B following Motor A in `Opposed` mode (they spin opposite directions but push the ball the same way). `Shooter` then drives `Hood` and `Turret` with setpoints received from `ShooterCalc`, which runs at 75 Hz on its own thread.

### Flywheel Control

Velocity is controlled via `VelocityTorqueCurrentFOC` (Slot 1). Torque-current feedback gives tighter velocity regulation under load vs plain voltage control. Commanding velocity = 0 switches to `CoastOut` instead, which prevents a stuck closed-loop error sitting at zero.

```java
void setShooterVelocity(double rotPerSec)   // direct
Command shootFromCalc()                      // uses m_calcFlywheelVelocityRotPerSec
```

`isShooterSpunUp()` returns true when the closed-loop error is within ±0.5 RPS of target.

### Ball Shot Detection

No optical sensor. The flywheel decelerates sharply when a ball engages the wheels, so we just watch the acceleration:

```
Threshold: acceleration <= -3.0 RPS/s
Gate: motor must actually be in velocity control mode
```

When a detection fires, a timer starts. The shot is "confirmed" after:
- The ball detection clears (acceleration recovers)
- 1.2 seconds have elapsed on the timer
- We're still controlling to shoot

This debounced trigger (`getBallShotDebounceTrg()`) is what tells the rest of the robot a ball has left, so it can advance the indexer or track shot count. Pretty neat for something with no sensor.

### Driver RPS Tweaks

Drivers can trim the computed flywheel speed without breaking anything

- **Dynamic**: ±5% of the calculated RPS
- **Static**: ±`kDriverRPSIncreaseD` per button press
- **Reset**: back to 0 on release

All tweaks are clamped to `[0, kShooterMaxRPSd]`. Gated behind `kAllowDriverRPSTweak` if you want to turn it off.
It's not being used rn but if we need to cope harder this is what we do :sunglasses:

### What periodic() Does

Every loop:
1. Caches turret position into a volatile field (ShooterCalc reads this from its thread).
2. Fetches the latest `ShotCalcOutputs` from the background thread.
3. Sends turret position + velocity feedforward to `Turret`.
4. Sends hood reference to `Hood` (if homed).
5. Applies driver RPS tweaks with clamping.
6. Refreshes the spun-up check.
7. Logs everything.

A `Tracer` is wired up for epoch timing diagnostics if you're hunting loop overruns. (hopefully syscore doesn't fail us)

### NT Tunable Overrides

- `/Shooter/shooterRPSOverride`: force a specific flywheel RPS (must be enabled in NT)
- `/Shooter/hoodRotsOverride`: force a specific hood position (must be enabled in NT)

Useful when building LERP tables. Leave them disabled during actual matches.

---

## Turret.java

The turret rotates on a Kraken X44 (FOC, on the Canivore bus) using `PositionVoltage` control. It also carries two absolute encoders, not for redundancy, but because one encoder alone can't tell you absolute position if the turret has more than one full rotation of range.

### Dual-Encoder Absolute Position Recovery

Two encoders are geared at different ratios:

- **Encoder A** (CANcoder): 10-tooth driving a 100-tooth output, so one full encoder reading every 36° of turret travel
- **Encoder B** (DutyCycleEncoder, DIO 3): 19-tooth driving a 100-tooth output, so one full encoder reading every ~18.95° of turret travel

Because these periods are incommensurate, at any given absolute turret angle, only one combination of (Encoder A reading, Encoder B reading) is possible. `calcTurretAngleLCM()` exploits this through Least Common Modulus:

1. Use Encoder A's reading to generate every candidate absolute angle within the turret's range (one per integer multiple of the 36° period).
2. For each candidate, predict what Encoder B *should* read at that angle.
3. Compare predicted vs actual Encoder B, wrapping the difference to ±180° to handle rollover.
4. Return the candidate with the smallest error.

This runs on construction (`homeTurret(true)`) and also every loop for logging. If both encoders agree, you can trust the position even after a power cycle. It also exists as an override button on the manipulator so that incase something goes wrong with that homing, we wont get screwed over :pray:

### Snapback Logic

The turret has soft limits but also a physical range. If the turret is commanded near one limit (e.g., it's at +0.4 rot heading toward +0.5 rot max) and the target wraps around the other side, `calcAzimuth` (in ShooterCalc) adds ±1 full rotation to the setpoint so the turret goes the short way around instead of slamming into a hardstop. `m_isSnappingBack` flags when the error is large enough that this is actively happening.

### Key Methods

```java
void homeTurret(boolean useLCM)               // true = dual-encoder fusion; false = kInitPosition
void setTurretPos(double rots, double velFF)  // main control path, called from Shooter.periodic()
void setTurretLock(boolean locked)            // lock at current position
void lockAndSetTurretLockPos(double lockPos)  // lock at a specific angle
void setIntaking(boolean intaking)            // freeze the turret during intake
```

---

## Hood.java

The hood controls launch angle. A TalonFXS drives it, and position is controlled via `PositionVoltage` (FOC disabled here).

### Homing

The hood homes by driving slowly into its physical lower limit and waiting for the stator current to spike above 5A (sustained for 0.125s via a `Debouncer`). When it stalls:
- Position is set to `kHoodAbsoluteMinRots`.
- Soft limits are re-applied.
- `m_isHoodHomed` flips to true.

There's a 5-second timeout in case something goes wrong. If it times out or gets interrupted, the hood brakes and logs a warning.

> Note: In the current build, the homing command is not set as a default command. The hood is initialized with `setPosition(0)` directly in the constructor and immediately considered homed. `hoodCurrentSenseHomingCmd()` is still available if you want to actually run the homing routine, and still exists as an override button.

### Key Methods

```java
void setHoodPos(double rots)           // set target position
Command hoodCurrentSenseHomingCmd()    // full homing routine (returns a command)
boolean isHoodHomed()
boolean atPosition()                   // within kHoodMaxErrD of setpoint
```

---

## How It All Fits Together

```
75 Hz Thread (ShooterCalc)
  reads: swerve state, volatile turret position
  writes: volatile ShotCalcOutputs

50 Hz Robot Loop (Shooter.periodic)
  reads: ShotCalcOutputs
  drives: Turret setpoint + velocity FF
  drives: Hood setpoint
  drives: Flywheel velocity (from operator commands, using calc output)
  reads: flywheel accel -> ball detection -> debounced shot trigger
```

The calc thread running at 75 Hz means shot math never blocks motor outputs, even when the iterative convergence takes a bit longer on a given cycle.
