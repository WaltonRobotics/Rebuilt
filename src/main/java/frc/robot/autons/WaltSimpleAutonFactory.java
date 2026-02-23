package frc.robot.autons;

import java.util.Set;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ShooterK;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.util.WaltLogger;
import frc.util.WaltLogger.IntLogger;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructre;
    private final Shooter m_shooter;
    private final AutoFactory m_autoFactory;


    public WaltSimpleAutonFactory(Superstructure superstructure, Shooter shooter, AutoFactory autoFactory) {
        m_superstructre = superstructure;
        m_shooter = shooter;
        m_autoFactory = autoFactory;
    }

    private Command preloadShot() {
        return Commands.sequence(
            Commands.waitSeconds(0.15),  //TODO: tune number
            m_superstructre.activateOuttake(ShooterK.kShooterMaxRPS)
        );
    }

    public Command rightOnePickup() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry("RightPickupOne"),
                preloadShot()
            ),
            m_superstructre.deactivateOuttake(),
            m_autoFactory.trajectoryCmd("RightPickupOne"),
            m_superstructre.activateIntake().withTimeout(2),
            Commands.parallel(
                m_superstructre.deactivateIntake(IntakeArmPosition.SAFE),
                m_autoFactory.resetOdometry("RightShootOne")
            ),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightShootOne"),
                Commands.sequence(
                    Commands.waitSeconds(1),  //TODO: tune number
                    m_superstructre.activateOuttake(ShooterK.kShooterMaxRPS)
                )
            )
        ).withName("rightOnePickup");
    }
}

