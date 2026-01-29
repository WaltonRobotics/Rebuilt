package frc.robot.subsystems;

import static frc.util.GeomUtil.toTransform2d;

import java.util.ArrayList;

import static frc.robot.Constants.ShooterK.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;

public class Shooter extends SubsystemBase {
    public class SimFuel {
        private double xi, yi, zi;
        private double xc, yc, zc;
        private double vx, vy, vz;
        private double initialTime;
       
        public SimFuel(
            double x,
            double y,
            double z,
            double vx,
            double vy,
            double vz,
            double initialTime
        ) {
            //initial position
            this.xi = x;
            this.yi = y;
            this.zi = z;

            //current position
            this.xc = x;
            this.yc = y;
            this.zc = z;

            //velocity
            this.vx = vx;
            this.vy = vy;
            this.vz = vz;
            this.initialTime = initialTime;
        }

        public void update(double currentTime) {
            if (zc > 0) {
                double elapsedTime = currentTime - initialTime;
                xc = xi + vx * elapsedTime;
                yc = yi + vy * elapsedTime;
                zc = zi + vz * elapsedTime - (9.8 / 2) * Math.pow(elapsedTime, 2);
            }
        }
    }

    ArrayList<SimFuel> simFuel = new ArrayList<SimFuel>();

    public void addFuel() {
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
        Pose3d turretPosition = new Pose3d(estimatedPose.transformBy(toTransform2d(kRobotToTurret)));


        double fuelSpeed = 
        double vx = 
            RobotState.getInstance().getRobotVelocity().vxMetersPerSecond
                * ;
        double vy = RobotState.getInstance().getRobotVelocity().vyMetersPerSecond;
        double vz = RobotState.getInstance().getRobotVelocity().vzMetersPerSecond;
        simFuel.add(
            new SimFuel(
                turretPosition.getX(),
                turretPosition.getY(), 
                turretPosition.getZ(), 
                0, 
                0, 
                0, 
                0))
    }

}
