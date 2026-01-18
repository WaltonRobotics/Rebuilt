package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

public class RobotState {
    /* FYI pulled from poofs 254 */

    private static RobotState m_instance;

    public static RobotState getInstance() {
        if (m_instance == null) {
            m_instance = new RobotState();
        }

        return m_instance;
    }


     /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Turret frame: origin is the center of the turret.
     */


     private Translation2d camera_to_goal = Translation2d

    
}
