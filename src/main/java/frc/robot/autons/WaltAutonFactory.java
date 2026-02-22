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

    //desired pose to go to after right fuel pickup
    private Pose2d m_postRightPickupNeutral;
    //desired pose to go to after left fuel pickup
    private Pose2d m_postLeftPickupNeutral;

    //desired pose to go to after right depot pickup
    //note: this name might change if a left depot path is created
    private Pose2d m_postPickupDepot;

    private String[] neutralCycle;
    private AutonSide m_side;

    public void setAutonSide(AutonSide side) {
        if (m_side != side) {
            m_side = side;

            neutralCycle = m_side.equals(AutonSide.RIGHT)
                                        ? new String[]{"RightToNeutral", "RightNeutralToShoot", "RightShootToNeutral"}
                                        : new String[]{"LeftToNeutral", "LeftNeutralToShoot", "LeftShootToNeutral"};
        }
    }

    public void setAlliance(boolean isRed) {
        m_postRightPickupNeutral = isRed ? AllianceFlipUtil.apply(AutonK.rightNeutralPose) : AutonK.rightNeutralPose;
        m_postPickupDepot = isRed ? AllianceFlipUtil.apply(AutonK.rightDepotPose) : AutonK.rightDepotPose;

        m_postLeftPickupNeutral = isRed ? AllianceFlipUtil.apply(AutonK.leftNeutralPose) : AutonK.leftNeutralPose;
    }

    /* CONSTRUCTOR */
    public WaltAutonFactory(AutoFactory autoFactory, Swerve drivetrain) {
        m_autoFactory = autoFactory;
        m_drivetrain = drivetrain;  

        setAutonSide(AutonSide.RIGHT);
    }

    /* AUTON COMMANDS */
    //---AUTON CONSTRUCTION
    public Command runTrajCmd(String traj) {
        return Commands.sequence(
            m_autoFactory.resetOdometry(traj),
            m_autoFactory.trajectoryCmd(traj)
        );
    }

    public Command pickupCmd(PickupLocation location) {
        Pose2d postPickupPose;

        switch (location) {
            case LEFT:
                postPickupPose = m_postLeftPickupNeutral;
                break;
            case RIGHT:
                postPickupPose = m_postRightPickupNeutral;
                break;
            case DEPOT:
                postPickupPose = m_postPickupDepot;
                break;
            default:
                postPickupPose = m_postRightPickupNeutral;
                break;
        }

        return Commands.sequence(
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
                runTrajCmd(neutralCycle[0]), //right to neutral
                pickupCmd(m_side.equals(AutonSide.RIGHT) ? PickupLocation.RIGHT : PickupLocation.LEFT),
                runTrajCmd(neutralCycle[1]) //neutral to shoot
            );
        }

        if (pickupTimes <= 0) { // Protect against invalid inputs
            return Commands.none();
        }

        ArrayList<Command> commandSequence = new ArrayList<Command>(); // Create the ArrayList to store commands

        commandSequence.add(createAutonSequence(pickupTimes - 1)); // Loop back recursively until it reaches the base case

        if (pickupTimes != 2) { // Code that is needed for pickupTimes >= 3 but not == 2
            commandSequence.add(runTrajCmd(neutralCycle[1])); //neutral to shoot
        }

        // Code required for both pickupTimes == 2 and != 2 
        commandSequence.add(runTrajCmd(neutralCycle[2])); //shoot to neutral
        commandSequence.add(pickupCmd(m_side.equals(AutonSide.RIGHT) ? PickupLocation.RIGHT : PickupLocation.LEFT));


        return Commands.sequence(commandSequence.toArray(new Command[commandSequence.size()]));// Return final command
    }

    //---AUTON OPTIONS
    /**
     * right pickup and shoot one time
     */
    public Command oneRightNeutralPickup() {
        setAutonSide(AutonSide.RIGHT);
        return createAutonSequence(1);
    }

    /**
     * right pick up two times and shoot once
     */
    public Command twoRightNeutralPickup() {
        setAutonSide(AutonSide.RIGHT);
        return createAutonSequence(2);
    }

    /**
     * right pick up three times and shoot twice
     */
    public Command threeRightNeutralPickup() {
        setAutonSide(AutonSide.RIGHT);
        return createAutonSequence(3);
    }

    /**
     * right pick up four times and shoot thrice
     */
    public Command fourRightNeutralPickup() {
        setAutonSide(AutonSide.RIGHT);
        return createAutonSequence(4);
    }

    /**
     * left pickup and shoot one time
     */
    public Command oneLeftNeutralPickup() {
        setAutonSide(AutonSide.LEFT);
        return createAutonSequence(1);
    }

    /**
     * left pick up two times and shoot once
     */
    public Command twoLeftNeutralPickup() {
        setAutonSide(AutonSide.LEFT);
        return createAutonSequence(2);
    }

    /**
     * left pick up three times and shoot twice
     */
    public Command threeLeftNeutralPickup() {
        setAutonSide(AutonSide.LEFT);
        return createAutonSequence(3);
    }

    /**
     * goes to depot once and shoots
     */
    public Command oneRightDepotPickup() {
        return Commands.sequence(
            runTrajCmd("RightToDepot"),
            
            pickupCmd(PickupLocation.DEPOT),

            runTrajCmd("RightDepotToShoot")
        );
    }

    /**
     * goes to outpost once, shoots, then goes to neutral to pickup
     */
    public Command oneRightOutpostPickup() {
        return Commands.sequence(
            runTrajCmd("RightToOutpost"),
            runTrajCmd("RightOutpostToNeutral")
        );
    }

    private enum PickupLocation {
        LEFT,
        RIGHT,
        DEPOT
    };

    private enum AutonSide {
        LEFT,
        RIGHT
    };
}
