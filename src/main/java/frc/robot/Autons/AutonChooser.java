package frc.robot.Autons;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonChooser {
    public static SendableChooser<String> m_chooser = new SendableChooser<>();

    //can prob move to constants?
    public static final String oneRightNeutralPickup = "oneNeutralPickup";
    public static final String twoRightNeutralPickup = "twoNeutralPickup";
    public static final String threeRightNeutralPickup = "threeNeutralPickup";

    public static NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();
    public static NetworkTable nte_autonChooser = nte_inst.getTable("AutonChooser");

    public static BooleanTopic BT_refreshChoice = nte_inst.getBooleanTopic("/AutonChooser/refreshChoice");
    public static StringTopic ST_autonName = nte_inst.getStringTopic("/AutonChooser/autonName");

    public static BooleanTopic BT_autonMade = nte_inst.getBooleanTopic("/AutonChooser/autonMade");

    public static BooleanPublisher pub_refreshChoice;
    public static StringPublisher pub_autonName;

    public static BooleanPublisher pub_autonMade;

    public static BooleanSubscriber sub_refreshChoice;
    public static StringSubscriber sub_autonName;

    public static BooleanSubscriber sub_autonMade;

    public static void initialize() {
        pub_refreshChoice = BT_refreshChoice.publish();
        pub_autonName = ST_autonName.publish();

        pub_autonMade = BT_autonMade.publish();

        sub_refreshChoice = BT_refreshChoice.subscribe(false);
        sub_autonName = ST_autonName.subscribe("No Auton Made");

        sub_autonMade = BT_autonMade.subscribe(false);

        pub_refreshChoice.setDefault(false);
        pub_autonName.setDefault("No Auton Made");

        pub_autonMade.setDefault(false);

        m_chooser.setDefaultOption("One Right Neutral Pickup", oneRightNeutralPickup);
        m_chooser.addOption("Two Right Neutral Pickup", twoRightNeutralPickup);
        m_chooser.addOption("Three Right Neutral Pickup ", threeRightNeutralPickup);

        SmartDashboard.putData(m_chooser);
    }
}

