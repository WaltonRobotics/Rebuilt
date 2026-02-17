package frc.util;

import static edu.wpi.first.units.Units.Degrees;

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

public class VisualSim {
    private final Intake m_intake;
    private final Indexer m_indexer;

    private final MechanismLigament2d m_deploy;
    private final MechanismLigament2d m_rollerVelocity;

    private final MechanismLigament2d m_spinnerVelocity;
    private final MechanismLigament2d m_exhaustVelocity;

    private final Rotation2d m_deployStartAngle;

    public VisualSim(Intake intake, Indexer indexer) {
        m_intake = intake;
        m_indexer = indexer;

        Mechanism2d intakeMech = new Mechanism2d(10,10);
        MechanismRoot2d intakeRoot = intakeMech.getRoot("intake", 4, 3);

        Mechanism2d indexerMech = new Mechanism2d(7, 15);
        MechanismRoot2d indexerRoot = indexerMech.getRoot("indexer", 3.5, 3.5);

        m_deploy = intakeRoot.append(new MechanismLigament2d("deploy", 4, 90));
        m_deployStartAngle = new Rotation2d(Degrees.of(90));
        m_rollerVelocity = intakeRoot.append(
            new MechanismLigament2d("intakeVelocity", 0, 0, 6, new Color8Bit(Color.kPeachPuff))
        );

        m_spinnerVelocity = indexerRoot.append(new MechanismLigament2d("spinner", 2, 0, 4, new Color8Bit(Color.kCornflowerBlue)));
        m_exhaustVelocity = indexerRoot.append(new MechanismLigament2d("exhaust", 2, 90, 4, new Color8Bit(Color.kAzure)));

        SmartDashboard.putData("IntakeMech2d", intakeMech);
        SmartDashboard.putData("IndexerMech2d", indexerMech);
    }  

    //TODO: there has got to be a way to overload these methods
    
    public Command setDeployPosition() {
        return Commands.run(
            () -> {
                m_deploy.setAngle(new Rotation2d(m_intake.getDeploy().getPosition().getValue()).plus(m_deployStartAngle));
            }
        );
    }

    public Command setRollerVelocity() {
        return Commands.run(
            () -> {
                m_rollerVelocity.setLength(m_intake.getRollers().getVelocity().getValueAsDouble()/10);
            }
        );
    }

    public Command setSpinnerVelocity() {
        return Commands.run(
            () -> {
                m_spinnerVelocity.setLength(m_indexer.getSpinner().getVelocity().getValueAsDouble()/10);
            }
        );
    }

    public Command setExhaustVelocity() {
        return Commands.run(
            () -> {
               m_exhaustVelocity.setLength(m_indexer.getExhaust().getVelocity().getValueAsDouble()/10);
            }
        );
        
    }
}
