package frc.robot.autons;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.subsystems.Swerve;
import frc.util.AllianceFlipUtil;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private final Swerve m_drivetrain;

    //desired pose to go to after fuel pickup
    private Pose2d postPickupNeutral;
    //desired pose to go to after depot pickup
    private Pose2d postPickupDepot;

    public WaltAutonFactory(AutoFactory autoFactory, Swerve drivetrain) {
        m_autoFactory = autoFactory;
        m_drivetrain = drivetrain;  
    }

    public void setAlliance(boolean isRed) {
        postPickupNeutral = isRed ? AllianceFlipUtil.flip(AutonK.neutralPose) : AutonK.neutralPose;
        postPickupDepot = isRed ? AllianceFlipUtil.flip(AutonK.depotPose) : AutonK.depotPose;
    }

    public Command runTrajCmd(String traj) {
        return Commands.sequence(
            m_autoFactory.resetOdometry(traj),
            m_autoFactory.trajectoryCmd(traj)
        );
    }

    public Command pickupCmd(boolean isNeutral) {
        Pose2d postPickupPose = isNeutral ? postPickupNeutral : postPickupDepot;

        return Commands.sequence(
            m_drivetrain.swerveToObject().withTimeout(1),
            m_drivetrain.toPose(postPickupPose).withTimeout(1)
        );
    }
    
    /**
     * pickup and shoot one time
     */
    public Command oneNeutralPickup() {
        return Commands.sequence(
            runTrajCmd("ToNeutral"),

            pickupCmd(true),

            runTrajCmd("NeutralToShoot")
        );
    }

    /**
     * pick up two times and shoot once
     */
    public Command twoNeutralPickup() {
        return Commands.sequence(
            oneNeutralPickup(),

            runTrajCmd("ShootToNeutral"),

            pickupCmd(true)
        );
    }

    /**
     * pick up three times and shoot twice
     */
    public Command threeNeutralPickup() {
        return Commands.sequence(
            twoNeutralPickup(),
            
            runTrajCmd("NeutralToShoot"),
            runTrajCmd("ShootToNeutral"),

            pickupCmd(true)
        );
    }

    /**
     * pick up four times and shoot thrice
     */
    public Command fourNeutralPickup() {
        return Commands.sequence(
            threeNeutralPickup(),

            runTrajCmd("NeutralToShoot"),
            runTrajCmd("ShootToNeutral"),

            pickupCmd(true)
        );
    }

    /**
     * goes to depot once and shoots
     */
    public Command oneDepotPickup() {
        return Commands.sequence(
            runTrajCmd("ToDepot"),
            
            pickupCmd(false),

            runTrajCmd("DepotToShoot")
        );
    }

    /**
     * goes to outpost once, shoots, then goes to neutral to pickup
     */
    public Command oneOutpostPickup() {
        return Commands.sequence(
            runTrajCmd("ToOutpost"),
            runTrajCmd("OutpostToNeutral")
        );
    }
}
