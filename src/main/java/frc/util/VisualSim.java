package frc.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class VisualSim {
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    private final MechanismLigament2d m_deployPosition;
    private final MechanismLigament2d m_rollerVelocity;

    private final MechanismLigament2d m_spinnerVelocity;
    private final MechanismLigament2d m_exhaustVelocity;

    private final MechanismLigament2d m_flywheelVelocity;
    private final MechanismLigament2d m_hoodPosition;
    private final MechanismLigament2d m_turretPosition;

    private final Rotation2d m_deployStartAngle;
    private final Rotation2d m_hoodStartAngle;
    private final Rotation2d m_turretStartAngle;

    public VisualSim(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;

        Mechanism2d intakeMech = new Mechanism2d(10,10);
        MechanismRoot2d intakeRoot = intakeMech.getRoot("intake", 4, 3);

        Mechanism2d indexerMech = new Mechanism2d(7, 15);
        MechanismRoot2d indexerRoot = indexerMech.getRoot("indexer", 3.5, 3.5);

        Mechanism2d shooterMech = new Mechanism2d(10, 15);
        MechanismRoot2d shooterRoot = shooterMech.getRoot("shooter", 5, 5);

        //--INTAKE
        m_deployPosition = intakeRoot.append(
            new MechanismLigament2d("deployPosition", 4, 90)
        );
        m_rollerVelocity = intakeRoot.append(
            new MechanismLigament2d("intakeVelocity", 0, 0, 4, new Color8Bit(Color.kPeachPuff))
        );
        m_deployStartAngle = new Rotation2d(Degrees.of(90));

        //--INDEXER
        m_spinnerVelocity = indexerRoot.append(
            new MechanismLigament2d("spinnerVelocity", 0, 0, 4, new Color8Bit(Color.kCornflowerBlue))
        );
        m_exhaustVelocity = indexerRoot.append(
            new MechanismLigament2d("exhaustVelocity", 0, 90, 4, new Color8Bit(Color.kAzure))
        );

        //--SHOOTER
        m_flywheelVelocity = shooterRoot.append(
            new MechanismLigament2d("flywheelVelocity", 0, 270, 4, new Color8Bit(Color.kFirebrick))
        );
        m_hoodPosition = shooterRoot.append(
            new MechanismLigament2d("hoodPosition", 4, 90, 4, new Color8Bit(Color.kDarkSalmon))
        );
        m_turretPosition = shooterRoot.append(
            new MechanismLigament2d("turretPosition", 4, 0, 4, new Color8Bit(Color.kTomato))
        );
        m_hoodStartAngle = new Rotation2d(Degrees.of(90));
        m_turretStartAngle = new Rotation2d(Degrees.of(0));

        SmartDashboard.putData("IntakeMech2d", intakeMech);
        SmartDashboard.putData("IndexerMech2d", indexerMech);
        SmartDashboard.putData("ShooterMech2d", shooterMech);
    } 

    //TODO: come up with better names
    private Command setVelocity(MechanismLigament2d simPart, TalonFX subsystem) {
        return Commands.run(
            () -> {
               simPart.setLength(subsystem.getVelocity().getValueAsDouble()/10);
            }
        );
    }

    //--INTAKE
    public Command setDeployPosition() {
        return Commands.run(
            () -> {
                m_deployPosition.setAngle(new Rotation2d(m_intake.getDeploy().getPosition().getValue()).plus(m_deployStartAngle));
            }
        );
    }

    public Command setRollerVelocity() {
        return setVelocity(m_rollerVelocity, m_intake.getRollers());
    }

    //--INDEXER
    public Command setSpinnerVelocity() {
        return setVelocity(m_spinnerVelocity, m_indexer.getSpinner());
    }

    public Command setExhaustVelocity() {
        return setVelocity(m_exhaustVelocity, m_indexer.getExhaust());
    }

    //--SHOOTER
    public Command setFlywheelVelocity() {
        return setVelocity(m_flywheelVelocity, m_shooter.getFlywheel());
    }

    //currently this will not work until hrehaan's fixies exist
    public Command setHoodPosition() {
        return Commands.run(
            () -> {
                m_hoodPosition.setAngle(new Rotation2d(m_shooter.getHoodEncoder().getPosition() * 2 * Math.PI).plus(m_hoodStartAngle));
            }
        );
    }

    public Command setTurretPosition() {
        return Commands.run(
            () -> {
                m_turretPosition.setAngle(new Rotation2d(m_shooter.getTurret().getPosition().getValue()).plus(m_turretStartAngle));
            }
        );
    }
}
