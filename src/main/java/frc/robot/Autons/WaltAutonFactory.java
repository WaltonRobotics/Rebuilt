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

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private final Swerve m_drivetrain;

    //desired pose to go to after fuel pickup
    private Pose2d postPickupNeutral = new Pose2d(Distance.ofRelativeUnits(6.924767017364502, Meter), 
        Distance.ofRelativeUnits(2.251265048980713, Meter), new Rotation2d(Math.PI));
    //desired pose to go to after depot pickup
    private Pose2d postPickupDepot = new Pose2d(Distance.ofRelativeUnits(1.1576627492904663, Meter), 
        Distance.ofRelativeUnits(5.958622932434082, Meter), new Rotation2d(0));
    

    public WaltAutonFactory(AutoFactory autoFactory, Swerve drivetrain) {
        m_autoFactory = autoFactory;
        m_drivetrain = drivetrain;
    }
    
    /**
     * pickup and shoot one time
     */
    public Command oneNeutralPickup() {
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isPresent() && allianceOpt.get().equals(Alliance.Red)) {
            postPickupNeutral = AllianceFlipUtil.flip(postPickupNeutral);
        }

        return Commands.sequence(
            m_autoFactory.resetOdometry("ToNeutral"),
            m_autoFactory.trajectoryCmd("ToNeutral"),

            m_drivetrain.swerveToObject().withTimeout(0.5),
            m_drivetrain.toPose(postPickupNeutral).withTimeout(0.5),

            m_autoFactory.resetOdometry("NeutralToShoot"),
            m_autoFactory.trajectoryCmd("NeutralToShoot")
        );
    }

    /**
     * pick up two times and shoot once
     */
    public Command twoNeutralPickup() {
        return Commands.sequence(
            oneNeutralPickup(),

            m_autoFactory.resetOdometry("ShootToNeutral"),
            m_autoFactory.trajectoryCmd("ShootToNeutral"),

            m_drivetrain.swerveToObject().withTimeout(0.5),
            m_drivetrain.toPose(postPickupNeutral).withTimeout(.5)
        );
    }

    /**
     * pick up three times and shoot twice
     */
    public Command threeNeutralPickup() {
        return Commands.sequence(
            twoNeutralPickup(),
            
            m_autoFactory.resetOdometry("NeutralToShoot"),
            m_autoFactory.trajectoryCmd("NeutralToShoot"),

            m_autoFactory.resetOdometry("ShootToNeutral"),
            m_autoFactory.trajectoryCmd("ShootToNeutral"),

            m_drivetrain.swerveToObject().withTimeout(.5),
            m_drivetrain.toPose(postPickupNeutral).withTimeout(.5)
        );
    }

    /**
     * pick up four times and shoot thrice
     */
    public Command fourNeutralPickup() {
        return Commands.sequence(
            threeNeutralPickup(),

            m_autoFactory.resetOdometry("NeutralToShoot"),
            m_autoFactory.trajectoryCmd("NeutralToShoot"),

            m_autoFactory.resetOdometry("ShootToNeutral"),
            m_autoFactory.trajectoryCmd("ShootToNeutral"),

            m_drivetrain.swerveToObject().withTimeout(.5),
            m_drivetrain.toPose(postPickupNeutral).withTimeout(.5)
        );
    }

    /**
     * goes to depot once and shoots
     */
    public Command oneDepotPickup() {
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isPresent() && allianceOpt.get().equals(Alliance.Red)) {
            postPickupDepot = AllianceFlipUtil.flip(postPickupDepot);
        }

        return Commands.sequence(
            m_autoFactory.resetOdometry("ToDepot"),
            m_autoFactory.trajectoryCmd("ToDepot"),
            
            //would we want this for depot?
            m_drivetrain.swerveToObject().withTimeout(3.5),
            m_drivetrain.toPose(postPickupDepot).withTimeout(3.5),

            m_autoFactory.resetOdometry("DepotToShoot"),
            m_autoFactory.trajectoryCmd("DepotToShoot")
        );
    }

    /**
     * goes to outpost once, shoots, then goes to neutral to pickup
     */
    public Command oneOutpostPickup() {
        return Commands.sequence(
            m_autoFactory.resetOdometry("ToOutpost"),
            m_autoFactory.trajectoryCmd("ToOutpost"),
            m_autoFactory.resetOdometry("OutpostToNeutral"),
            m_autoFactory.trajectoryCmd("OutpostToNeutral")
        );
    }
}
