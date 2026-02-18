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

    public WaltAutonFactory(AutoFactory autoFactory, Swerve drivetrain) {
        m_autoFactory = autoFactory;
        m_drivetrain = drivetrain;  

        neutralCycle = new String[3];
    }

    public void setAlliance(boolean isRed) {
        m_postRightPickupNeutral = isRed ? AllianceFlipUtil.flip(AutonK.rightNeutralPose) : AutonK.rightNeutralPose;
        m_postPickupDepot = isRed ? AllianceFlipUtil.flip(AutonK.rightDepotPose) : AutonK.rightDepotPose;

        m_postLeftPickupNeutral = isRed ? AllianceFlipUtil.flip(AutonK.leftNeutralPose) : AutonK.leftNeutralPose;
    }

    public Command runTrajCmd(String traj) {
        return Commands.sequence(
            m_autoFactory.resetOdometry(traj),
            m_autoFactory.trajectoryCmd(traj)
        );
    }

    public Command pickupCmd(boolean isNeutral, boolean isRight) {
        Pose2d postPickupPose = isNeutral ? m_postLeftPickupNeutral : m_postPickupDepot;

        //b/c there are currently no left auton paths that go to the depot, if the path is to depot, it will default to the right post depot pickup
        if (isRight) {
            postPickupPose = isNeutral ? m_postRightPickupNeutral : m_postPickupDepot;
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
    private Command createAutonSequence(int pickupTimes, boolean isRight) {
        neutralCycle = isRight ? new String[]{"RightToNeutral", "RightNeutralToShoot", "RightShootToNeutral"} : 
                                 new String[]{"LeftToNeutral", "LeftNeutralToShoot", "LeftShootToNeutral"};

        if (pickupTimes == 1) { // Base case when pickupTimes = 1 in the recursive loop
            return Commands.sequence(
                runTrajCmd(neutralCycle[0]), //right to neutral
                pickupCmd(true, isRight),
                runTrajCmd(neutralCycle[1]) //neutral to shoot
            );
        }

        if (pickupTimes <= 0) { // Protect against invalid inputs
            return Commands.none();
        }

        ArrayList<Command> commandSequence = new ArrayList<Command>(); // Create the ArrayList to store commands

        commandSequence.add(createAutonSequence(pickupTimes - 1, isRight)); // Loop back recursively until it reaches the base case

        if (pickupTimes != 2) { // Code that is needed for pickupTimes >= 3 but not == 2
            commandSequence.add(runTrajCmd(neutralCycle[1])); //neutral to shoot
        }

        // Code required for both pickupTimes == 2 and != 2 
        commandSequence.add(runTrajCmd(neutralCycle[2])); //shoot to neutral
        commandSequence.add(pickupCmd(true, isRight));

        return Commands.sequence(commandSequence.toArray(new Command[commandSequence.size()]));// Return final command
    }

    /**
     * right pickup and shoot one time
     */
    public Command oneRightNeutralPickup() {
        return createAutonSequence(1, true);
    }

    /**
     * right pick up two times and shoot once
     */
    public Command twoRightNeutralPickup() {
        return createAutonSequence(2, true);
    }

    /**
     * right pick up three times and shoot twice
     */
    public Command threeRightNeutralPickup() {
        return createAutonSequence(3, true);
    }

    /**
     * right pick up four times and shoot thrice
     */
    public Command fourRightNeutralPickup() {
        return createAutonSequence(4, true);
    }

    /**
     * left pickup and shoot one time
     */
    public Command oneLeftNeutralPickup() {
        return createAutonSequence(1, false);
    }

    /**
     * left pick up two times and shoot once
     */
    public Command twoLeftNeutralPickup() {
        return createAutonSequence(2, false);
    }

    /**
     * left pick up three times and shoot twice
     */
    public Command threeLeftNeutralPickup() {
        return createAutonSequence(3, false);
    }

    /**
     * goes to depot once and shoots for the right side
     */
    public Command oneRightDepotPickup() {
        return Commands.sequence(
            runTrajCmd("RightToDepot"),
            
            pickupCmd(false, true),

            runTrajCmd("RightDepotToShoot")
        );
    }

    /**
     * goes to outpost once, shoots, then goes to neutral to pickup for the right side
     */
    public Command oneRightOutpostPickup() {
        return Commands.sequence(
            runTrajCmd("RightToOutpost"),
            runTrajCmd("RightOutpostToNeutral")
        );
    }
}
