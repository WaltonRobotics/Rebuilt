package frc.robot.subsystems;

import static frc.robot.Constants.ShooterK.kTurretTalonFXConfiguration;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.WaltLogger;
import frc.robot.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterPrimary = new TalonFX(20);
    private final TalonFX shooterSecondary = new TalonFX(21);
    private final TalonFX turret = new TalonFX(22);

    private final MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble("Shooter", "turretPositionRots");

     private final DCMotorSim m_turretSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1),
            0.104506595,
            41.66666666
        ),
        DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    public Shooter() {
        shooterSecondary.setControl(new Follower(20, MotorAlignmentValue.Opposed));
        turret.getConfigurator().apply(kTurretTalonFXConfiguration);
        
        initSim();
    }

    private void initSim() {
        var turretFXSim = turret.getSimState();
        turretFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        turretFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    public Command shoot(DoubleSubscriber sub_Shooter) {
        return runEnd(() -> {
            shooterPrimary.set(sub_Shooter.get(0));
        }, () -> {
            shooterPrimary.set(0);
        });
    }

    public Command setTurretPositionCmd(DoubleSubscriber sub_Turret) {
        return runOnce(() -> turret.setControl(m_MMVRequest.withPosition(sub_Turret.get())));
    }

    @Override
    public void periodic() {
        log_turretPositionRots.accept(turret.getPosition().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var turretFXSim = turret.getSimState();

        m_turretSim.setInputVoltage(turretFXSim.getMotorVoltage());
        m_turretSim.update(0.02);

        turretFXSim.setRawRotorPosition(m_turretSim.getAngularPositionRotations() * 41.666666);
        turretFXSim.setRotorVelocity(m_turretSim.getAngularVelocity().times(41.666666));
        turretFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
