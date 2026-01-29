package frc.robot.subsystems.shooter;

import static frc.util.GeomUtil.toTransform2d;

import java.util.ArrayList;

import static frc.robot.Constants.ShooterK.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.util.WaltLogger;
import frc.util.WaltLogger.*;

public class Shooter extends SubsystemBase {
    private ArrayList<SimFuel> m_simFuel = new ArrayList<SimFuel>();
    
    private final Pose3dLogger log_latestFuelTrajectory = WaltLogger.logPose3d(kLogTab, "latestFuelTrajectory");
    private final Pose3dLogger log_fuelPoses = WaltLogger.logPose3d(kLogTab, "fuelPoses");

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

    public void addFuel() {
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
        //double turretPosition = Turret.getCurrentPosition(); (? does the reader think this will work/be designed ?)
        Pose3d turretPosition = new Pose3d(estimatedPose.transformBy(toTransform2d(kRobotToTurret))); //highly temporary, but i think the turret position would just be reading encoder values
        double hoodAngle = 75; //degrees | Hood.getAngle() or something like that(i believe it will be a val 0-90?)

        double fuelSpeed = 10; //meters per second (also just a dummy value)
        
        double vVertical = fuelSpeed + Math.sin(hoodAngle);
        double vHorizontal = fuelSpeed + Math.cos(hoodAngle);


        double vx = 
            RobotState.getInstance().getRobotVelocity().vxMetersPerSecond
                + vHorizontal
                    * Math.cos(turretPosition.getRotation().getZ()); //for now it will be this, but otherwise we can just put in the double turretPosition no?
        double vy = 
            RobotState.getInstance().getRobotVelocity().vyMetersPerSecond
                + vHorizontal
                    * Math.sin(turretPosition.getRotation().getZ()); //again same thing here ^^
        double vz = vVertical;

        m_simFuel.add(
            new SimFuel(
                turretPosition.getX(),
                turretPosition.getY(), 
                turretPosition.getZ(), 
                vx, 
                vy, 
                vz, 
                Timer.getFPGATimestamp()));
        
        ArrayList<Pose3d> latestFuelTrajectory = new ArrayList<>();
        boolean onGround = false;
        double elapsedTime = 0;
        while (!onGround) {
            int idx = 0;
            SimFuel fuel = new SimFuel(
                turretPosition.getX(),
                turretPosition.getY(),
                turretPosition.getZ(),
                vx,
                vy,
                vz,
                Timer.getFPGATimestamp()
            );
            fuel.update(Timer.getFPGATimestamp() + elapsedTime);
            elapsedTime += 4; //very dummy value | I HAVE NO CLUE WHAT THE FRICK THE FUELTRAJECTORYTIMEINTERVAL IS WHAT THE HELL 
            if (fuel.zc > 0) {
                latestFuelTrajectory.add(
                    new Pose3d(
                        fuel.xc, fuel.yc, fuel.zc, new Rotation3d(0,0,0)
                        )
                );
                idx++;
            } else {
                onGround = true;
            }
            log_latestFuelTrajectory.accept(latestFuelTrajectory.get(idx)); //does this work? we shall see!
        }
    }
    
    public void periodic() {
        ArrayList<Pose3d> fuelPoses = new ArrayList<>();
        for (SimFuel fuel : m_simFuel) {
            int idx = 0;
            fuel.update(Timer.getFPGATimestamp());
            if (fuel.zc <= 0) {
                m_simFuel.remove(fuel);
            } else {
                fuelPoses.add(
                    new Pose3d(fuel.xc, fuel.yc, fuel.zc, new Rotation3d(0,0,0))
                );
                log_fuelPoses.accept(fuelPoses.get(idx));
                idx++;
            }
            
        }
    }

}
