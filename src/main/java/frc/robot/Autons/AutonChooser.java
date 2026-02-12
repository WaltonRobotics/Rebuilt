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
    static public SendableChooser<String> m_chooser = new SendableChooser<>();

    static public final String oneRightNeutralPickup = "oneNeutralPickup";
    static public final String twoRightNeutralPickup = "twoNeutralPickup";
    static public final String threeRightNeutralPickup = "threeNeutralPickup";
    static public final String noneSelected = "noAutonSelected";

    static public NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();
    static public NetworkTable nte_autonChooser = nte_inst.getTable("AutonChooser");

    //TOPICS
    static public BooleanTopic BT_refreshChoice = nte_inst.getBooleanTopic("/AutonChooser/refreshChoice");
    static public StringTopic ST_autonName = nte_inst.getStringTopic("/AutonChooser/autonName");

    static public BooleanTopic BT_autonMade = nte_inst.getBooleanTopic("/AutonChooser/autonMade");

    //PUBLISHERS
    static public BooleanPublisher pub_refreshChoice;
    static public StringPublisher pub_autonName;

    static public BooleanPublisher pub_autonMade;

    //SUBSCRIBERS
    static public BooleanSubscriber sub_refreshChoice;
    static public StringSubscriber sub_autonName;

    static public BooleanSubscriber sub_autonMade;

    static public void initialize() {
        pub_refreshChoice = BT_refreshChoice.publish();
        pub_autonName = ST_autonName.publish();

        pub_autonMade = BT_autonMade.publish();

        sub_refreshChoice = BT_refreshChoice.subscribe(false);
        sub_autonName = ST_autonName.subscribe("No Auton Made");

        sub_autonMade = BT_autonMade.subscribe(false);

        pub_refreshChoice.setDefault(false);
        pub_autonName.setDefault("No Auton Made");

        pub_autonMade.setDefault(false);

        m_chooser.setDefaultOption("No Auton Selected", noneSelected);
        m_chooser.addOption("One Right Neutral Pickup", oneRightNeutralPickup);
        m_chooser.addOption("Two Right Neutral Pickup", twoRightNeutralPickup);
        m_chooser.addOption("Three Right Neutral Pickup ", threeRightNeutralPickup);

        SmartDashboard.putData(m_chooser);
    }
}

