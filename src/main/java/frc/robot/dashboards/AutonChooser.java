package frc.robot.dashboards;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autons.WaltSimpleAutonFactory;
public class AutonChooser {
    public static NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();
    public static SendableChooser<Command> m_chooser = new SendableChooser<>();
    public static WaltSimpleAutonFactory m_simpleAutonFactory;
    public static Command m_autonomousCommand;

    public static void initialize(WaltSimpleAutonFactory simpleAutonFactory) {
        m_simpleAutonFactory = simpleAutonFactory;

        m_chooser.setDefaultOption("None Selected", Commands.none());
        m_chooser.addOption("One Right Neutral Pickup", m_simpleAutonFactory.rightOneSweep());
        m_chooser.addOption("One Left Neutral Pickup", m_simpleAutonFactory.leftOneSweep());
        m_chooser.addOption("Two Right Neutral Pickup", m_simpleAutonFactory.rightTwoSweep());
        m_chooser.addOption("Right Outpost to Shoot", m_simpleAutonFactory.rightOutpostToShoot());
        m_chooser.addOption("Right Depot To Shoot", m_simpleAutonFactory.rightDepotToShoot());

        SmartDashboard.putData(m_chooser);
    }
}
