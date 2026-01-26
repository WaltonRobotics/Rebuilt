package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.WaltLogger;
import frc.robot.util.WaltLogger.DoubleLogger;

public class Turret extends SubsystemBase {

    private final TalonFX m_motor = new TalonFX(11);

    private final MotionMagicVoltage m_MMVReq = new MotionMagicVoltage(Angle.ofBaseUnits(0, Units.Degrees));

    private final DoubleLogger log_turretSimPositionMysteryUnits = WaltLogger.logDouble("Turret Logs", "Position in Mystery Units");
    private final DoubleLogger log_turretSimPositionRotations = WaltLogger.logDouble("Turret Logs", "Position in Rotations");

    // private final FlywheelSim turretSim = new FlywheelSim(null, null, null);

    // Motor Positions (for max/min assume exclusive) NOT rotations (mystery units)
    private final double kMaxPos = 135;
    private final double kMinPos = -821;
    public final double kHomingPos = 0;
    private final double kFrontFacingPos = -330;
    private final double kUnitsPerRotation = 1360; // 1360 units per rotation
    private final double kUnitsPerDegree = kUnitsPerRotation / 360.0;

    private double currentPos;

    public Turret() {
        m_motor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withPeakForwardDutyCycle(0.1)
                .withPeakReverseDutyCycle(0.1)
            )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withForwardSoftLimitThreshold(Rotations.of(0.78))
                .withReverseSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(Rotations.of(0.02))
            )
        );
    }

    public double getPosRotations() {
        return m_motor.getPosition().getValue().in(Units.Rotations);
    }

    public double getPosDegrees() {
        return m_motor.getPosition().getValue().in(Units.Degrees);
    }

    public double getPosMysteryUnits() { // placeholder name bc we dont know the units
        return currentPos; 
    } 
    
    /**
     * Rotates turret to specified position.
     * @param pos Position in [mystery unit]
     */
    public void moveToPosMysteryUnits(double pos) {
        if (kMinPos < pos && kMaxPos > pos) {
            currentPos = pos;
            m_motor.setControl(m_MMVReq.withPosition(Angle.ofBaseUnits(pos / kUnitsPerDegree, Units.Degrees)));
        }
    }

    /**
     * Rotates turret to specified position.
     * @param pos Position in rotations
     */
    public void moveToPosRotations(double rots) {
        if (kMinPos / kUnitsPerRotation < rots && kMaxPos / kUnitsPerRotation > rots) {
            currentPos = rots * kUnitsPerRotation;
            m_motor.setControl(m_MMVReq.withPosition(Angle.ofBaseUnits(rots / 360, Units.Degrees)));
        }
    }

    /**
     * Rotates turret to specified position.
     * @param pos Position in degrees
     */
    public void moveToPosDegrees(double degs) {
        if (kMinPos / kUnitsPerDegree < degs && kMaxPos / kUnitsPerDegree > degs) {
            currentPos = degs * kUnitsPerDegree;
            m_motor.setControl(m_MMVReq.withPosition(Angle.ofBaseUnits(degs, Units.Degrees)));
        }
    }

    public Command spin(DoubleSubscriber sub_Turret) {
        return run(() -> {
            moveToPosMysteryUnits(sub_Turret.getAsDouble());
        });
    }

    public void moveToMax() {
        moveToPosMysteryUnits(kMaxPos - 0.1);
    }

    public void moveToMin() {
        moveToPosMysteryUnits(kMinPos + 0.1);
    }

    public void moveToHome() {
        moveToPosMysteryUnits(kHomingPos);
    }

    public void moveToFrontFacing() {
        moveToPosMysteryUnits(kFrontFacingPos);
    }

    /*
    @Override
    public void simulationPeriodic() {
        TalonFXSimState m_simMotor = new TalonFXSimState(m_motor);
        moveToPosRotations(0.5);
        m_motor.getSimState().setRawRotorPosition(getPosRotations());
        // moveToPosRotations(0.5);
        logValue(getPosRotations());
    }

    private void logValue(double val) {
        log_turretSimPositionMysteryUnits.accept(val * kUnitsPerRotation);
        log_turretSimPositionRotations.accept(val);
    }
        */
}