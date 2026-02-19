package frc.robot.autons;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WaltDashboard {
    public static NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();

    public class TestingDashboard {
        public static NetworkTable nte_testingDashboard = nte_inst.getTable("TestingDashboard");

        public static NetworkTable nte_shooter = nte_inst.getTable("Shooter");
        public static NetworkTable nte_indexer = nte_inst.getTable("Indexer");
        public static NetworkTable nte_intake = nte_inst.getTable("Intake");

        /* TOPICS */
        //---SHOOTER
        public static DoubleTopic DT_shooterVelocityRPS = nte_inst.getDoubleTopic("/TestingDashboard/shooterVelocityRPS");
        public static DoubleTopic DT_turretPositionRots = nte_inst.getDoubleTopic("/TestingDashboard/turretPositionRots");
        public static DoubleTopic DT_hoodPositionDegs = nte_inst.getDoubleTopic("/TestingDashboard/hoodPositionDegs");

        //---INDEXER
        public static DoubleTopic DT_spindexerVelocityRPS = nte_inst.getDoubleTopic("/TestingDashboard/spindexerVelocityRPS");
        public static DoubleTopic DT_tunnelVelocityRPS = nte_inst.getDoubleTopic("/TestingDashboard/tunnelVelocityRPS");

        //---INTAKE
        public static DoubleTopic DT_intakeArmPositionDegs = nte_inst.getDoubleTopic("/TestingDashboard/intakeArmPositionDegs");
        public static DoubleTopic DT_intakeRollersVelocityRPS = nte_inst.getDoubleTopic("/TestingDashboard/intakeRollersVelocityRPS");

        //---"ALLOW CHANGES" SWITCHES
        public static BooleanTopic BT_letShooterVelocityRPSChange = nte_inst.getBooleanTopic("/TestingDashboard/letShooterVelocityRPSChange");
        public static BooleanTopic BT_letTurretPositionRotsChange = nte_inst.getBooleanTopic("/TestingDashboard/letTurretPositionRotsChange");
        public static BooleanTopic BT_letHoodPositionDegsChange = nte_inst.getBooleanTopic("/TestingDashboard/letHoodPositionDegsChange");

        public static BooleanTopic BT_letSpindexerVelocityRPSChange = nte_inst.getBooleanTopic("/TestingDashboard/letSpindexerVelocityRPSChange");
        public static BooleanTopic BT_letTunnelVelocityRPSChange = nte_inst.getBooleanTopic("/TestingDashboard/letTunnelVelocityRPSChange");

        public static BooleanTopic BT_letIntakeArmPositionDegsChange = nte_inst.getBooleanTopic("/TestingDashboard/letIntakeArmPositionDegsChange");
        public static BooleanTopic BT_letIntakeRollersVelocityRPSChange = nte_inst.getBooleanTopic("/TestingDashboard/letIntakeRollersVelocityRPSChange");

        /* PUBLISHERS */
        //---SHOOTER
        public static DoublePublisher pub_shooterVelocityRPS;
        public static DoublePublisher pub_turretPositionRots;
        public static DoublePublisher pub_hoodPositionDegs;

        //---INDEXER
        public static DoublePublisher pub_spindexerVelocityRPS;
        public static DoublePublisher pub_tunnelVelocityRPS;

        //---INTAKE
        public static DoublePublisher pub_intakeArmPositionDegs;
        public static DoublePublisher pub_intakeRollersVelocityRPS;

        //---"ALLOW CHANGES" SWITCHES
        public static BooleanPublisher pub_letShooterVelocityRPSChange;
        public static BooleanPublisher pub_letTurretPositionRotsChange;
        public static BooleanPublisher pub_letHoodPositionDegsChange;

        public static BooleanPublisher pub_letSpindexerVelocityRPSChange;
        public static BooleanPublisher pub_letTunnelVelocityRPSChange;

        public static BooleanPublisher pub_letIntakeArmPositionDegsChange;
        public static BooleanPublisher pub_letIntakeRollersVelocityRPSChange;

        /* SUBSCRIBERS */
        //---SHOOTER
        public static DoubleSubscriber sub_shooterVelocityRPS;
        public static DoubleSubscriber sub_turretPositionRots;
        public static DoubleSubscriber sub_hoodPositionDegs;

        //---INDEXER
        public static DoubleSubscriber sub_spindexerVelocityRPS;
        public static DoubleSubscriber sub_tunnelVelocityRPS;

        //---INTAKE
        public static DoubleSubscriber sub_intakeArmPositionDegs;
        public static DoubleSubscriber sub_intakeRollersVelocityRPS;

        //---"ALLOW CHANGES" SWITCHES
        public static BooleanSubscriber sub_letShooterVelocityRPSChange;
        public static BooleanSubscriber sub_letTurretPositionRotsChange;
        public static BooleanSubscriber sub_letHoodPositionDegsChange;

        public static BooleanSubscriber sub_letSpindexerVelocityRPSChange;
        public static BooleanSubscriber sub_letTunnelVelocityRPSChange;

        public static BooleanSubscriber sub_letIntakeArmPositionDegsChange;
        public static BooleanSubscriber sub_letIntakeRollersVelocityRPSChange;

        public static void initialize() {
            //---SHOOTER
            pub_shooterVelocityRPS = DT_shooterVelocityRPS.publish();
            pub_turretPositionRots = DT_turretPositionRots.publish();
            pub_hoodPositionDegs = DT_hoodPositionDegs.publish();

            pub_shooterVelocityRPS.setDefault(0);
            pub_turretPositionRots.setDefault(0);
            pub_hoodPositionDegs.setDefault(0);

            sub_shooterVelocityRPS = DT_shooterVelocityRPS.subscribe(0);
            sub_turretPositionRots = DT_turretPositionRots.subscribe(0);
            sub_hoodPositionDegs = DT_hoodPositionDegs.subscribe(0);

            //---INDEXER
            pub_spindexerVelocityRPS = DT_spindexerVelocityRPS.publish();
            pub_tunnelVelocityRPS = DT_tunnelVelocityRPS.publish();

            pub_spindexerVelocityRPS.setDefault(0);
            pub_tunnelVelocityRPS.setDefault(0);

            sub_spindexerVelocityRPS = DT_spindexerVelocityRPS.subscribe(0);
            sub_tunnelVelocityRPS = DT_tunnelVelocityRPS.subscribe(0);

            //---INTAKE
            pub_intakeArmPositionDegs = DT_intakeArmPositionDegs.publish();
            pub_intakeRollersVelocityRPS = DT_intakeRollersVelocityRPS.publish();

            pub_intakeArmPositionDegs.setDefault(0);
            pub_intakeRollersVelocityRPS.setDefault(0);

            sub_intakeArmPositionDegs = DT_intakeArmPositionDegs.subscribe(0);
            sub_intakeRollersVelocityRPS = DT_intakeRollersVelocityRPS.subscribe(0);

            //---"ALLOW CHANGES" SWITCHES
            pub_letShooterVelocityRPSChange = BT_letShooterVelocityRPSChange.publish();
            pub_letTurretPositionRotsChange = BT_letTurretPositionRotsChange.publish();
            pub_letHoodPositionDegsChange = BT_letHoodPositionDegsChange.publish();
            pub_letSpindexerVelocityRPSChange = BT_letSpindexerVelocityRPSChange.publish();
            pub_letTunnelVelocityRPSChange = BT_letTunnelVelocityRPSChange.publish();
            pub_letIntakeArmPositionDegsChange = BT_letIntakeArmPositionDegsChange.publish();
            pub_letIntakeRollersVelocityRPSChange = BT_letIntakeRollersVelocityRPSChange.publish();

            pub_letShooterVelocityRPSChange.setDefault(false);
            pub_letTurretPositionRotsChange.setDefault(false);
            pub_letHoodPositionDegsChange.setDefault(false);
            pub_letSpindexerVelocityRPSChange.setDefault(false);
            pub_letTunnelVelocityRPSChange.setDefault(false);
            pub_letIntakeArmPositionDegsChange.setDefault(false);
            pub_letIntakeRollersVelocityRPSChange.setDefault(false);

            sub_letShooterVelocityRPSChange = BT_letShooterVelocityRPSChange.subscribe(false);
            sub_letTurretPositionRotsChange = BT_letTurretPositionRotsChange.subscribe(false);
            sub_letHoodPositionDegsChange = BT_letHoodPositionDegsChange.subscribe(false);
            sub_letSpindexerVelocityRPSChange = BT_letSpindexerVelocityRPSChange.subscribe(false);
            sub_letTunnelVelocityRPSChange = BT_letTunnelVelocityRPSChange.subscribe(false);
            sub_letIntakeArmPositionDegsChange = BT_letIntakeArmPositionDegsChange.subscribe(false);
            sub_letIntakeRollersVelocityRPSChange = BT_letIntakeRollersVelocityRPSChange.subscribe(false);
        }
    }

    public class AutonChooser {
        /* CLASS VARIABLES */
        public static SendableChooser<String> m_chooser = new SendableChooser<>();
        public static NetworkTable nte_autonChooser = nte_inst.getTable("AutonChooser");

        //---AUTON PATH NAMES
        public static final String oneRightNeutralPickup = "oneNeutralPickup";
        public static final String twoRightNeutralPickup = "twoNeutralPickup";
        public static final String threeRightNeutralPickup = "threeNeutralPickup";

        /* TOPICS */
        public static StringTopic ST_autonName = nte_inst.getStringTopic("/AutonChooser/autonName");
        public static BooleanTopic BT_autonMade = nte_inst.getBooleanTopic("/AutonChooser/autonMade");
        public static BooleanTopic BT_makeAuton = nte_inst.getBooleanTopic("/AutonChooser/makeAuton");

        /* PUBLISHERS */
        public static StringPublisher pub_autonName;
        public static BooleanPublisher pub_autonMade;
        public static BooleanPublisher pub_makeAuton;

        /* SUBSCRIBERS */
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
            m_chooser.addOption("Three Right Neutral Pickup ", threeRightNeutralPickup);

            SmartDashboard.putData(m_chooser);
        }
    }
}

