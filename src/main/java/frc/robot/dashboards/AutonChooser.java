package frc.robot.dashboards;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.autons.WaltSimpleAutonFactory;
import frc.robot.autons.WaltSimpleAutonFactory.WaltPathAndCommand;
public class AutonChooser {
    public static NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();
    public static SendableChooser<WaltPathAndCommand> m_chooser = new SendableChooser<>();
    public static WaltSimpleAutonFactory m_simpleAutonFactory;
    public static Command m_autonomousCommand;

    private static final WaltPathAndCommand auton_none = new WaltPathAndCommand("", Commands.none());

    private static WaltPathAndCommand auton_oneCycleGoInNow_Right(WaltSimpleAutonFactory simpleAutonFactory) {
        return new WaltPathAndCommand(AutonK.kRightSweepPathName, simpleAutonFactory.oneCycleGoInNow(false));
    }
    private static WaltPathAndCommand auton_oneCycleGoInNow_Left(WaltSimpleAutonFactory simpleAutonFactory) {
        return new WaltPathAndCommand(AutonK.kLeftSweepPathName, simpleAutonFactory.oneCycleGoInNow(true));
    }


    public static void initialize(WaltSimpleAutonFactory simpleAutonFactory) {
        m_simpleAutonFactory = simpleAutonFactory;

        m_chooser.setDefaultOption("None Selected", auton_none);
        m_chooser.addOption("Fast One Right Neutral Pickup", auton_oneCycleGoInNow_Right(m_simpleAutonFactory));
        m_chooser.addOption("Fast One Left Neutral Pickup", auton_oneCycleGoInNow_Left(m_simpleAutonFactory));

        SmartDashboard.putData(m_chooser);
        m_chooser.onChange((var pathAndCmd) -> {
            var now = RobotController.getFPGATime();
            m_simpleAutonFactory.m_autoFactory.cache().loadTrajectory(pathAndCmd.pathName);
            var elapsedMs = (RobotController.getFPGATime() - now) / 1000.0;
            System.out.println("");
            System.out.println("");
            System.out.println("=================================================================================");
            System.out.println("=============== AUTON PATH \"" + pathAndCmd.pathName +  "\" LOADED ==============");
            System.out.println("=================================================================================");
            var tookStr = System.out.format("Took %.2f ms", elapsedMs);
            System.out.println(tookStr);

        });
    }

    public static void cleanup() {
        SendableRegistry.remove(m_chooser);
    }
}
