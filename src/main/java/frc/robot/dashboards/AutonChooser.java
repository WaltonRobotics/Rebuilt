package frc.robot.dashboards;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
        return new WaltPathAndCommand(AutonK.kRightSweepPathName, simpleAutonFactory.oneCycleGoInNow(false, false));
    }
    private static WaltPathAndCommand auton_oneCycleGoInNow_Left(WaltSimpleAutonFactory simpleAutonFactory) {
        return new WaltPathAndCommand(AutonK.kLeftSweepPathName, simpleAutonFactory.oneCycleGoInNow(true, false));
    }

    private static WaltPathAndCommand auton_oneCycleGoInNow_LeftOptimize(WaltSimpleAutonFactory simpleAutonFactory) {
        return new WaltPathAndCommand(AutonK.kLeftOptimizedSweepPathName, simpleAutonFactory.oneCycleGoInNow(true, true));
    }

    private static WaltPathAndCommand auton_oneCycleGoInNow_RightOptimize(WaltSimpleAutonFactory simpleAutonFactory) {
        return new WaltPathAndCommand(AutonK.kRightOptimizedSweepPathName, simpleAutonFactory.oneCycleGoInNow(false, true));
    }

    private static WaltPathAndCommand auton_twoRightSweep(WaltSimpleAutonFactory simpleAutonFactory) {
        return new WaltPathAndCommand(AutonK.kRightSweepPathName, simpleAutonFactory.rightTwoSweep(false));
    }

    public static void initialize(WaltSimpleAutonFactory simpleAutonFactory) {
        m_simpleAutonFactory = simpleAutonFactory;

        m_chooser.setDefaultOption("None Selected", auton_none);
        m_chooser.addOption("Fast One Right Neutral Pickup", auton_oneCycleGoInNow_Right(m_simpleAutonFactory));
        m_chooser.addOption("Fast One Left Neutral Pickup", auton_oneCycleGoInNow_Left(m_simpleAutonFactory));
        m_chooser.addOption("Optimized One Left Neutral Pickup", auton_oneCycleGoInNow_LeftOptimize(simpleAutonFactory));
        m_chooser.addOption("Optimized One Right Neutral Pickup", auton_oneCycleGoInNow_RightOptimize(simpleAutonFactory));
        m_chooser.addOption("Two Right Sweep", auton_twoRightSweep(simpleAutonFactory));

        SmartDashboard.putData(m_chooser);
    }

    public static Command getPreheater() {
        if (m_simpleAutonFactory == null) {
            return Commands.print("Tried to preheat before factory init!");
        }
        return m_simpleAutonFactory.preheater().ignoringDisable(true);
    }

    public static void cleanup() {
        SendableRegistry.remove(m_chooser);
    }
}
