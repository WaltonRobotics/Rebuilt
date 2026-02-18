package frc.robot.autons;

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

    //---AUTON PATH NAMES
    public static final String oneRightNeutralPickup = "oneRightNeutralPickup";
    public static final String twoRightNeutralPickup = "twoRightNeutralPickup";
    public static final String threeRightNeutralPickup = "threeRightNeutralPickup";
    public static final String oneLeftNeutralPickup = "oneLeftNeutralPickup";
    public static final String twoLeftNeutralPickup = "twoLeftNeutralPickup";
    public static final String threeLeftNeutralPickup = "threeLeftNeutralPickup";

    //---TOPICS
    public static NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();
    public static NetworkTable nte_autonChooser = nte_inst.getTable("AutonChooser");

    //---TOPICS
    public static StringTopic ST_autonName = nte_inst.getStringTopic("/AutonChooser/autonName");
    public static BooleanTopic BT_autonMade = nte_inst.getBooleanTopic("/AutonChooser/autonMade");
    public static BooleanTopic BT_makeAuton = nte_inst.getBooleanTopic("/AutonChooser/makeAuton");

    //---PUBLISHERS
    public static StringPublisher pub_autonName;
    public static BooleanPublisher pub_autonMade;
    public static BooleanPublisher pub_makeAuton;

    //---SUBSCRIBERS
    public static StringSubscriber sub_autonName;
    public static BooleanSubscriber sub_autonMade;
    public static BooleanSubscriber sub_makeAuton;

    public static void initialize() {
        pub_autonName = ST_autonName.publish();

        pub_autonMade = BT_autonMade.publish();
        pub_makeAuton = BT_makeAuton.publish();

        pub_autonName.setDefault("No Auton Made");

        pub_autonMade.setDefault(false);

        sub_autonName = ST_autonName.subscribe("No Auton Made");

        sub_autonMade = BT_autonMade.subscribe(false);
        sub_makeAuton = BT_makeAuton.subscribe(false);

        m_chooser.setDefaultOption("One Right Neutral Pickup", oneRightNeutralPickup);
        m_chooser.addOption("Two Right Neutral Pickup", twoRightNeutralPickup);
        m_chooser.addOption("Three Right Neutral Pickup", threeRightNeutralPickup);
        m_chooser.addOption("One Left Neutral Pickup", oneLeftNeutralPickup);
        m_chooser.addOption("Two Left Neutral Pickup", twoLeftNeutralPickup);
        m_chooser.addOption("Three Left Neutral Pickup", threeLeftNeutralPickup);

        SmartDashboard.putData(m_chooser);
    }
}

