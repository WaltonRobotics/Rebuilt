package frc.robot.autons;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.subsystems.Swerve;
import frc.util.AllianceFlipUtil;

public class WaltAutonFactory {
    /* CLASS VARIABLES */
    private final AutoFactory m_autoFactory;
    private final Swerve m_drivetrain;

    private Pose2d m_postPickupNeutral;     //desired pose to go to after fuel pickup
    private Pose2d m_postPickupDepot;    //desired pose to go to after depot pickup

    /* CONSTRUCTOR */
    public WaltAutonFactory(AutoFactory autoFactory, Swerve drivetrain) {
        m_autoFactory = autoFactory;
        m_drivetrain = drivetrain;  
    }

    public void setAlliance(boolean isRed) {
        m_postPickupNeutral = isRed ? AllianceFlipUtil.flip(AutonK.neutralPose) : AutonK.neutralPose;
        m_postPickupDepot = isRed ? AllianceFlipUtil.flip(AutonK.depotPose) : AutonK.depotPose;
    }

    /* AUTON COMMANDS */
    //---AUTON CONSTRUCTION
    public Command runTrajCmd(String traj) {
        return Commands.sequence(
            m_autoFactory.resetOdometry(traj),
            m_autoFactory.trajectoryCmd(traj)
        );
    }

    public Command pickupCmd(boolean isNeutral) {
        Pose2d postPickupPose = isNeutral ? m_postPickupNeutral : m_postPickupDepot;

        return Commands.sequence(
            Commands.print("-------------------------POST: " + postPickupPose.toString() + "------------------------------"),
            Commands.print("------------------------------NEUTRAL: " + m_postPickupNeutral.toString() + "--------------------------------"),
            m_drivetrain.swerveToObject().withTimeout(1),
            m_drivetrain.toPose(postPickupPose).withTimeout(1)
        );
    }

    /**
     * Pass in an integer pickupTimes to create a sequence of commands for an auton that
     * picks up that many times (and shoots accordingly). Uses recursion.
     * @param pickupTimes The number of times the robot pick ups from the neutral zone
     * @return A command group containing a sequence of commands to execute the auton
     */
    private Command createAutonSequence(int pickupTimes) {
        if (pickupTimes == 1) { // Base case when pickupTimes = 1 in the recursive loop
            return Commands.sequence(
                runTrajCmd("ToNeutral"),
                pickupCmd(true),
                runTrajCmd("NeutralToShoot")
            );
        }

        if (pickupTimes <= 0) { // Protect against invalid inputs
            return Commands.none();
        }

        ArrayList<Command> commandSequence = new ArrayList<Command>(); // Create the ArrayList to store commands

        commandSequence.add(createAutonSequence(pickupTimes - 1)); // Loop back recursively until it reaches the base case

        if (pickupTimes != 2) { // Code that is needed for pickupTimes >= 3 but not == 2
            commandSequence.add(runTrajCmd("NeutralToShoot"));
        }

        commandSequence.add(runTrajCmd("ShootToNeutral")); // Code required for both pickupTimes == 2 and != 2 
        commandSequence.add(pickupCmd(true));

        return Commands.sequence(commandSequence.toArray(new Command[commandSequence.size()]));// Return final command
    }

    //---AUTON OPTIONS
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
        return createAutonSequence(2);
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
