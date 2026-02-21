package frc.util;

import static edu.wpi.first.units.Units.Degrees;

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
    // private final MechanismLigament2d m_hoodPosition;
    // private final MechanismLigament2d m_turretPosition;

    private final Rotation2d m_intakeArmStartAngle;
    // private final Rotation2d m_hoodStartAngle;
    // private final Rotation2d m_turretStartAngle;

    public WaltVisualSim(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;

        Mechanism2d intakeMech = new Mechanism2d(10,10);
        MechanismRoot2d intakeRoot = intakeMech.getRoot("intake", 4.55, 0.1);

        Mechanism2d indexerMech = new Mechanism2d(7, 15);
        MechanismRoot2d indexerRoot = indexerMech.getRoot("indexer", 3.5, 0.15);

        // Mechanism2d turretMech = new Mechanism2d(10, 20);
        // MechanismRoot2d turretRoot = turretMech.getRoot("turret", 5, 0.7);
        Mechanism2d shooterMech = new Mechanism2d(10, 20);
        MechanismRoot2d shooterRoot = shooterMech.getRoot("shooter", 5, 0.7);

        //---INTAKE
        m_intakeArmPosition = intakeRoot.append(
            new MechanismLigament2d("intakeArmPosition", .4, 90, 2, new Color8Bit(Color.kOrange))
        );
        m_intakeRollerVelocity = m_intakeArmPosition.append(
            new MechanismLigament2d("intakeRollerVelocity", 0, 0, 2, new Color8Bit(Color.kPeachPuff))
        );
        m_intakeArmStartAngle = new Rotation2d(Degrees.of(90));

        //---INDEXER
        m_spindexerVelocity = indexerRoot.append(
            new MechanismLigament2d("spindexerVelocity", 0, 180, 2, new Color8Bit(Color.kCornflowerBlue))
        );
        m_tunnelVelocity = indexerRoot.append(
            new MechanismLigament2d("tunnelVelocity", 0, 90, 2, new Color8Bit(Color.kAzure))
        );

        //---SHOOTER
        // m_turretPosition = turretRoot.append(
        //     new MechanismLigament2d("turretPosition", .4, 180, 2, new Color8Bit(Color.kTomato))
        // );
        // m_hoodPosition = shooterRoot.append(
        //     new MechanismLigament2d("hoodPosition", .4, 0, 2, new Color8Bit(Color.kDarkSalmon))
        // );
        m_shooterVelocity = shooterRoot.append(
            new MechanismLigament2d("shooterVelocity", 0, 90, 2, new Color8Bit(Color.kFirebrick))
        );
        
        // m_hoodStartAngle = new Rotation2d(Degrees.of(0));
        // m_turretStartAngle = new Rotation2d(Degrees.of(180));

        SmartDashboard.putData("IntakeMech2d", intakeMech);
        SmartDashboard.putData("IndexerMech2d", indexerMech);
        SmartDashboard.putData("ShooterMech2d", shooterMech);
        // SmartDashboard.putData("TurretMech2d", turretMech);
    } 

    /**
     * Sets the length of the simPart based on the velocity
     * @param simPart A part of the simulated robot
     * @param subsystemMotor The motor for the part.
     * @param divisor Scales down to be proportional to the robot
     * @return A command that sets the length of the simPart to the motor velocity / 10.
     */
    private Command setVelocity(MechanismLigament2d simPart, TalonFX subsystemMotor, double divisor) {
        if (divisor != 0) {
            return Commands.run(
                () -> {
                    simPart.setLength(subsystemMotor.getVelocity().getValueAsDouble() / divisor);
                }
            );
        } else {
            return Commands.none();
        }
    }

    //---INTAKE
    /**
     * Sets the angle of the intake arm sim object.
     * @return A command that sets the angle of the intake arm to the current position plus the start angle.
     */
    public Command setIntakeArmPosition() {
        return Commands.run(
            () -> {
                m_intakeArmPosition.setAngle(new Rotation2d(m_intake.getIntakeArmMotor().getPosition().getValue()).plus(m_intakeArmStartAngle));
            }
        );
    }
    
    /**
     * Sets the intake roller sim object velocity.
     * 
     * @return A call to setVelocity that takes the intakeRollerVelocity and the intake rollers.
     */
    public Command setIntakeRollerVelocity() {
        return setVelocity(m_intakeRollerVelocity, m_intake.getIntakeRollers(), 100);
    }

    //---INDEXER
    /**
     * Sets the spindexer sim object velocity.
     * @return A call to setVelocity that takes spindexerVelocity and the spindexer.
     */
    public Command setSpindexerVelocity() {
        return setVelocity(m_spindexerVelocity, m_indexer.getSpindexer(), 100);
    }

    /**
     * Sets the tunnel sim object velocity.
     * @return A call to setVelocity that takes tunnelVelocity and the tunnel.
     */
    public Command setTunnelVelocity() {
        return setVelocity(m_tunnelVelocity, m_indexer.getTunnel(), 200);
    }

    //---SHOOTER
    /**
     * Sets the shooter sim object velocity.
     * @return A call to setVelocity that takes shooterVelocity and the shooter.
     */
    public Command setShooterVelocity() {
        return setVelocity(m_shooterVelocity, m_shooter.getShooter(), 100);
    }

    //NOTE: Made turret in3D based on a robot pose commented out old turret code in case we want it as some point
    /**
     * Sets the hood sim object position.
     * @return A command that sets the angle to the start angle minus the current position of the shooter.
     */
    // public Command setHoodPosition() {
    //     return Commands.run(
    //         () -> {
    //             m_hoodPosition.setAngle(m_hoodStartAngle.minus(new Rotation2d(Rotations.of(m_shooter.getHoodSimEncoder().getAngularPositionRotations()))));
    //         }
    //     );
    // }

    /**
     * Sets the turret position.
     * @return A command that sets the angle to the position of the turret plus the start angle.
     */
    // public Command setTurretPosition() {
    //     return Commands.run(
    //         () -> {
    //             m_turretPosition.setAngle(new Rotation2d(m_shooter.getTurret().getPosition().getValue()).plus(m_turretStartAngle));
    //         }
    //     );
    // }
}
