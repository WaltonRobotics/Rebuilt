package frc.robot.autons;

import java.util.ArrayList;
import java.util.Arrays;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    private ArrayList<Command> autonSequence;

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

    public Command createAutonSequence(int pickupTimes) {
        if (pickupTimes == 1) {
            return Commands.sequence(
                runTrajCmd("ToNeutral"),
                pickupCmd(true),
                runTrajCmd("NeutralToShoot")
            );
        }

        if (pickupTimes <= 0) {
            return Commands.none();
        }

        ArrayList<Command> commandSequence = new ArrayList<Command>();

        commandSequence.add(createAutonSequence(pickupTimes - 1));

        if (pickupTimes == 2) {
            commandSequence.add(Commands.sequence(
                runTrajCmd("ShootToNeutral"),         
                pickupCmd(true)
            ));
        } else {
            commandSequence.add(Commands.sequence(
                runTrajCmd("NeutralToShoot"),
                runTrajCmd("ShootToNeutral"),
                pickupCmd(true)
            ));
        }

        return Commands.sequence(commandSequence.toArray(new Command[commandSequence.size()]));
    }

    /**
     * pickup and shoot one time
     */
    public Command oneNeutralPickup() {
        return createAutonSequence(1);
    }

    /**
     * pick up two times and shoot once
     */
    public Command twoNeutralPickup() {
        return createAutonSequence(1);
    }

    /**
     * pick up three times and shoot twice
     */
    public Command threeNeutralPickup() {
        return createAutonSequence(3);
    }

    /**
     * pick up four times and shoot thrice
     */
    public Command fourNeutralPickup() {
        return createAutonSequence(4);
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
