package frc.robot.Autons;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.util.AllianceFlipUtil;

public class AutonFactory {
    private final AutoFactory m_autoFactory;
    private final Swerve m_drivetrain;

    public AutonFactory(AutoFactory autoFactory, Swerve drivetrain) {
        m_autoFactory = autoFactory;
        m_drivetrain = drivetrain;
    }
    
    /**
     * pickup and shoot one time
     * @return
     */
    public Command simpleAuton() {
        // Simple Auton (hardcoded)
        final var idle = new SwerveRequest.Idle();

        //can i somehow pull this directly from choreo and not have to do this
        Pose2d postPickup = new Pose2d(Distance.ofRelativeUnits(6.924767017364502, Meter), 
            Distance.ofRelativeUnits(2.251265048980713, Meter), new Rotation2d(Math.PI));

        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isPresent() && allianceOpt.get().equals(Alliance.Red)) {
            postPickup = AllianceFlipUtil.flip(postPickup);
        }

        return Commands.sequence(
            m_autoFactory.resetOdometry("OnePickup"),
            m_autoFactory.trajectoryCmd("OnePickup"),

            //i feel like there is a better way to do this other than Commands.race()
            Commands.race(
                m_drivetrain.swerveToObject(),
                Commands.waitSeconds(3)
            ),
            Commands.race(
                m_drivetrain.toPose(postPickup),
                Commands.waitSeconds(3)
            ),
            
            m_autoFactory.resetOdometry("ToShoot"),
            m_autoFactory.trajectoryCmd("ToShoot")
        );
    }
    
}
