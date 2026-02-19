package frc.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

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

public class WaltVisualSim {
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    private final MechanismLigament2d m_intakeArmPosition;
    private final MechanismLigament2d m_intakeRollerVelocity;

    private final MechanismLigament2d m_spindexerVelocity;
    private final MechanismLigament2d m_tunnelVelocity;

    private final MechanismLigament2d m_shooterVelocity;
    private final MechanismLigament2d m_hoodPosition;
    private final MechanismLigament2d m_turretPosition;

    private final Rotation2d m_intakeArmStartAngle;
    private final Rotation2d m_hoodStartAngle;
    private final Rotation2d m_turretStartAngle;

    public WaltVisualSim(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;

        Mechanism2d intakeMech = new Mechanism2d(10,10);
        MechanismRoot2d intakeRoot = intakeMech.getRoot("intake", 4, 3);

        Mechanism2d indexerMech = new Mechanism2d(7, 15);
        MechanismRoot2d indexerRoot = indexerMech.getRoot("indexer", 3.5, 3.5);

        Mechanism2d shooterMech = new Mechanism2d(10, 20);
        MechanismRoot2d shooterRoot = shooterMech.getRoot("shooter", 5, 5);

        //---INTAKE
        m_intakeArmPosition = intakeRoot.append(
            new MechanismLigament2d("intakeArmPosition", 4, 90)
        );
        m_intakeRollerVelocity = intakeRoot.append(
            new MechanismLigament2d("intakeRollerVelocity", 0, 0, 4, new Color8Bit(Color.kPeachPuff))
        );
        m_intakeArmStartAngle = new Rotation2d(Degrees.of(90));

        //---INDEXER
        m_spindexerVelocity = indexerRoot.append(
            new MechanismLigament2d("spindexerVelocity", 0, 0, 4, new Color8Bit(Color.kCornflowerBlue))
        );
        m_tunnelVelocity = indexerRoot.append(
            new MechanismLigament2d("tunnelVelocity", 0, 90, 4, new Color8Bit(Color.kAzure))
        );

        //---SHOOTER
        m_shooterVelocity = shooterRoot.append(
            new MechanismLigament2d("shooterVelocity", 0, 90, 4, new Color8Bit(Color.kFirebrick))
        );
        m_hoodPosition = shooterRoot.append(
            new MechanismLigament2d("hoodPosition", 4, 180, 4, new Color8Bit(Color.kDarkSalmon))
        );
        m_turretPosition = shooterRoot.append(
            new MechanismLigament2d("turretPosition", 4, 0, 4, new Color8Bit(Color.kTomato))
        );
        m_hoodStartAngle = new Rotation2d(Degrees.of(180));
        m_turretStartAngle = new Rotation2d(Degrees.of(0));

        SmartDashboard.putData("IntakeMech2d", intakeMech);
        SmartDashboard.putData("IndexerMech2d", indexerMech);
        SmartDashboard.putData("ShooterMech2d", shooterMech);
    } 

    private Command setVelocity(MechanismLigament2d simPart, TalonFX subsystemMotor) {
        return Commands.run(
            () -> {
               simPart.setLength(subsystemMotor.getVelocity().getValueAsDouble()/10);
            }
        );
    }

    //---INTAKE
    public Command setIntakeArmPosition() {
        return Commands.run(
            () -> {
                m_intakeArmPosition.setAngle(new Rotation2d(m_intake.getIntakeArmMotor().getPosition().getValue()).plus(m_intakeArmStartAngle));
            }
        );
    }

    public Command setIntakeRollerVelocity() {
        return setVelocity(m_intakeRollerVelocity, m_intake.getIntakeRollers());
    }

    //---INDEXER
    public Command setSpindexerVelocity() {
        return setVelocity(m_spindexerVelocity, m_indexer.getSpindexer());
    }

    public Command setTunnelVelocity() {
        return setVelocity(m_tunnelVelocity, m_indexer.getTunnel());
    }

    //---SHOOTER
    public Command setShooterVelocity() {
        return setVelocity(m_shooterVelocity, m_shooter.getShooter());
    }

    public Command setHoodPosition() {
        return Commands.run(
            () -> {
                m_hoodPosition.setAngle(m_hoodStartAngle.minus(new Rotation2d(Rotations.of(m_shooter.getHoodSimEncoder().getAngularPositionRotations()))));
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
