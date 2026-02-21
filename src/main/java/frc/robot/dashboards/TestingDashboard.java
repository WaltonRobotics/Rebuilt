package frc.robot.dashboards;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TestingDashboard {
    public static NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();
    public static NetworkTable nte_testingDashboard = nte_inst.getTable("TestingDashboard");

    /* TOPICS */
    //---SHOOTER
    public static DoubleTopic DT_shooterVelocityRPS = nte_inst.getDoubleTopic("/TestingDashboard/shooterVelocityRPS");
    public static DoubleTopic DT_turretPositionRots = nte_inst.getDoubleTopic("/TestingDashboard/turretPositionRots");
    public static DoubleTopic DT_hoodPositionDegs = nte_inst.getDoubleTopic("/TestingDashboard/hoodPositionDegs");

    //---INDEXER
    public static DoubleTopic DT_spindexerVelocityRPS = nte_inst.getDoubleTopic("/TestingDashboard/spindexerVelocityRPS");
    public static DoubleTopic DT_tunnelVelocityRPS = nte_inst.getDoubleTopic("/TestingDashboard/tunnelVelocityRPS");

    //---INTAKE
    public static DoubleTopic DT_intakeArmPositionRots = nte_inst.getDoubleTopic("/TestingDashboard/intakeArmPositionRots");
    public static DoubleTopic DT_intakeRollersVelocityRPS = nte_inst.getDoubleTopic("/TestingDashboard/intakeRollersVelocityRPS");

    //---"ALLOW CHANGES" SWITCHES
    public static BooleanTopic BT_letShooterVelocityRPSChange = nte_inst.getBooleanTopic("/TestingDashboard/letShooterVelocityRPSChange");
    public static BooleanTopic BT_letTurretPositionRotsChange = nte_inst.getBooleanTopic("/TestingDashboard/letTurretPositionRotsChange");
    public static BooleanTopic BT_letHoodPositionDegsChange = nte_inst.getBooleanTopic("/TestingDashboard/letHoodPositionDegsChange");

    public static BooleanTopic BT_letSpindexerVelocityRPSChange = nte_inst.getBooleanTopic("/TestingDashboard/letSpindexerVelocityRPSChange");
    public static BooleanTopic BT_letTunnelVelocityRPSChange = nte_inst.getBooleanTopic("/TestingDashboard/letTunnelVelocityRPSChange");

    public static BooleanTopic BT_letIntakeArmPositionRotsChange = nte_inst.getBooleanTopic("/TestingDashboard/letIntakeArmPositionRotsChange");
    public static BooleanTopic BT_letIntakeRollersVelocityRPSChange = nte_inst.getBooleanTopic("/TestingDashboard/letIntakeRollersVelocityRPSChange");

    //---BRAKE SWITCHES
    public static BooleanTopic BT_shooterBrake = nte_inst.getBooleanTopic("/TestingDashboard/shooterBrake");
    public static BooleanTopic BT_turretBrake = nte_inst.getBooleanTopic("/TestingDashboard/turretBrake");
    public static BooleanTopic BT_hoodPositionBrake = nte_inst.getBooleanTopic("/TestingDashboard/hoodPositionBrake");
    public static BooleanTopic BT_intakeArmPositionBrake = nte_inst.getBooleanTopic("/TestingDashboard/intakeArmPositionBrake");
    public static BooleanTopic BT_intakeRollersBrake = nte_inst.getBooleanTopic("/TestingDashboard/intakeRollersBrake");
    public static BooleanTopic BT_spindexerBrake = nte_inst.getBooleanTopic("/TestingDashboard/spindexerBrake");    
    public static BooleanTopic BT_tunnelBrake = nte_inst.getBooleanTopic("/TestingDashboard/tunnelBrake");

    /* PUBLISHERS */
    //---SHOOTER
    public static DoublePublisher pub_shooterVelocityRPS;
    public static DoublePublisher pub_turretPositionRots;
    public static DoublePublisher pub_hoodPositionDegs;

    //---INDEXER
    public static DoublePublisher pub_spindexerVelocityRPS;
    public static DoublePublisher pub_tunnelVelocityRPS;

    //---INTAKE
    public static DoublePublisher pub_intakeArmPositionRots;
    public static DoublePublisher pub_intakeRollersVelocityRPS;

    //---"ALLOW CHANGES" SWITCHES
    public static BooleanPublisher pub_letShooterVelocityRPSChange;
    public static BooleanPublisher pub_letTurretPositionRotsChange;
    public static BooleanPublisher pub_letHoodPositionDegsChange;

    public static BooleanPublisher pub_letSpindexerVelocityRPSChange;
    public static BooleanPublisher pub_letTunnelVelocityRPSChange;

    public static BooleanPublisher pub_letIntakeArmPositionRotsChange;
    public static BooleanPublisher pub_letIntakeRollersVelocityRPSChange;

    //---BRAKE SWITCHES
    public static BooleanPublisher pub_shooterBrake;
    public static BooleanPublisher pub_turretBrake;
    public static BooleanPublisher pub_hoodPositionBrake;
    public static BooleanPublisher pub_intakeArmPositionBrake;
    public static BooleanPublisher pub_intakeRollersBrake;
    public static BooleanPublisher pub_spindexerBrake;
    public static BooleanPublisher pub_tunnelBrake;

    /* SUBSCRIBERS */
    //---SHOOTER
    public static DoubleSubscriber sub_shooterVelocityRPS;
    public static DoubleSubscriber sub_turretPositionRots;
    public static DoubleSubscriber sub_hoodPositionDegs;

    //---INDEXER
    public static DoubleSubscriber sub_spindexerVelocityRPS;
    public static DoubleSubscriber sub_tunnelVelocityRPS;

    //---INTAKE
    public static DoubleSubscriber sub_intakeArmPositionRots;
    public static DoubleSubscriber sub_intakeRollersVelocityRPS;

    //---"ALLOW CHANGES" SWITCHES
    public static BooleanSubscriber sub_letShooterVelocityRPSChange;
    public static BooleanSubscriber sub_letTurretPositionRotsChange;
    public static BooleanSubscriber sub_letHoodPositionDegsChange;

    public static BooleanSubscriber sub_letSpindexerVelocityRPSChange;
    public static BooleanSubscriber sub_letTunnelVelocityRPSChange;

    public static BooleanSubscriber sub_letIntakeArmPositionRotsChange;
    public static BooleanSubscriber sub_letIntakeRollersVelocityRPSChange;

    //---BRAKE SWITCHES
    public static BooleanSubscriber sub_shooterBrake;
    public static BooleanSubscriber sub_turretBrake;
    public static BooleanSubscriber sub_hoodPositionBrake;
    public static BooleanSubscriber sub_intakeArmPositionBrake;
    public static BooleanSubscriber sub_intakeRollersBrake;
    public static BooleanSubscriber sub_spindexerBrake;
    public static BooleanSubscriber sub_tunnelBrake;

    /* TRIGGERS */
    public static Trigger trg_letShooterVelocityRPSChange;
    public static Trigger trg_letTurretPositionRotsChange;
    public static Trigger trg_letHoodPositionDegsChange;

    public static Trigger trg_letSpindexerVelocityRPSChange;
    public static Trigger trg_letTunnelVelocityRPSChange;

    public static Trigger trg_letIntakeArmPositionRotsChange;
    public static Trigger trg_letIntakeRollersVelocityRPSChange;

    //---BRAKE SWITCHES
    public static Trigger trg_shooterBrake;
    public static Trigger trg_turrentBrake;
    public static Trigger trg_hoodPositionBrake;
    public static Trigger trg_intakeArmPositionBrake;
    public static Trigger trg_intakeRollersBrake;
    public static Trigger trg_spindexerBrake;
    public static Trigger trg_tunnelBrake;

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
        pub_intakeArmPositionRots = DT_intakeArmPositionRots.publish();
        pub_intakeRollersVelocityRPS = DT_intakeRollersVelocityRPS.publish();

        pub_intakeArmPositionRots.setDefault(0);
        pub_intakeRollersVelocityRPS.setDefault(0);

        sub_intakeArmPositionRots = DT_intakeArmPositionRots.subscribe(0);
        sub_intakeRollersVelocityRPS = DT_intakeRollersVelocityRPS.subscribe(0);

        //---"ALLOW CHANGES" SWITCHES
        pub_letShooterVelocityRPSChange = BT_letShooterVelocityRPSChange.publish();
        pub_letTurretPositionRotsChange = BT_letTurretPositionRotsChange.publish();
        pub_letHoodPositionDegsChange = BT_letHoodPositionDegsChange.publish();
        pub_letSpindexerVelocityRPSChange = BT_letSpindexerVelocityRPSChange.publish();
        pub_letTunnelVelocityRPSChange = BT_letTunnelVelocityRPSChange.publish();
        pub_letIntakeArmPositionRotsChange = BT_letIntakeArmPositionRotsChange.publish();
        pub_letIntakeRollersVelocityRPSChange = BT_letIntakeRollersVelocityRPSChange.publish();

        pub_letShooterVelocityRPSChange.setDefault(false);
        pub_letTurretPositionRotsChange.setDefault(false);
        pub_letHoodPositionDegsChange.setDefault(false);
        pub_letSpindexerVelocityRPSChange.setDefault(false);
        pub_letTunnelVelocityRPSChange.setDefault(false);
        pub_letIntakeArmPositionRotsChange.setDefault(false);
        pub_letIntakeRollersVelocityRPSChange.setDefault(false);

        sub_letShooterVelocityRPSChange = BT_letShooterVelocityRPSChange.subscribe(false);
        sub_letTurretPositionRotsChange = BT_letTurretPositionRotsChange.subscribe(false);
        sub_letHoodPositionDegsChange = BT_letHoodPositionDegsChange.subscribe(false);
        sub_letSpindexerVelocityRPSChange = BT_letSpindexerVelocityRPSChange.subscribe(false);
        sub_letTunnelVelocityRPSChange = BT_letTunnelVelocityRPSChange.subscribe(false);
        sub_letIntakeArmPositionRotsChange = BT_letIntakeArmPositionRotsChange.subscribe(false);
        sub_letIntakeRollersVelocityRPSChange = BT_letIntakeRollersVelocityRPSChange.subscribe(false);

        //---BRAKE SWITCHES
        pub_shooterBrake = BT_shooterBrake.publish();
        pub_turretBrake = BT_turretBrake.publish();
        pub_hoodPositionBrake = BT_hoodPositionBrake.publish();
        pub_intakeArmPositionBrake = BT_intakeArmPositionBrake.publish();
        pub_intakeRollersBrake = BT_intakeRollersBrake.publish();
        pub_spindexerBrake = BT_spindexerBrake.publish();
        pub_tunnelBrake = BT_tunnelBrake.publish();

        pub_shooterBrake.setDefault(false);
        pub_turretBrake.setDefault(false);
        pub_hoodPositionBrake.setDefault(false);
        pub_intakeArmPositionBrake.setDefault(false);
        pub_intakeRollersBrake.setDefault(false);
        pub_spindexerBrake.setDefault(false);
        pub_tunnelBrake.setDefault(false);

        sub_shooterBrake = BT_shooterBrake.subscribe(false);
        sub_turretBrake = BT_turretBrake.subscribe(false);
        sub_hoodPositionBrake = BT_hoodPositionBrake.subscribe(false);
        sub_intakeArmPositionBrake = BT_intakeArmPositionBrake.subscribe(false);
        sub_intakeRollersBrake = BT_intakeRollersBrake.subscribe(false);
        sub_spindexerBrake = BT_spindexerBrake.subscribe(false);
        sub_tunnelBrake = BT_tunnelBrake.subscribe(false);

        //---TRIGGERS
        trg_letShooterVelocityRPSChange = new Trigger(() -> sub_letShooterVelocityRPSChange.get());
        trg_letTurretPositionRotsChange = new Trigger(() -> sub_letTurretPositionRotsChange.get());
        trg_letHoodPositionDegsChange = new Trigger(() -> sub_letHoodPositionDegsChange.get());

        trg_letSpindexerVelocityRPSChange = new Trigger(() -> sub_letSpindexerVelocityRPSChange.get());
        trg_letTunnelVelocityRPSChange = new Trigger(() -> sub_letTunnelVelocityRPSChange.get());

        trg_letIntakeArmPositionRotsChange = new Trigger(() -> sub_letIntakeArmPositionRotsChange.get());
        trg_letIntakeRollersVelocityRPSChange = new Trigger(() -> sub_letIntakeRollersVelocityRPSChange.get());

        //---BRAKE SWITCHES
        trg_shooterBrake = new Trigger(() -> sub_shooterBrake.get());
        trg_turrentBrake = new Trigger(() -> sub_turretBrake.get());
        trg_hoodPositionBrake = new Trigger(() -> sub_hoodPositionBrake.get());
        trg_intakeArmPositionBrake = new Trigger(() -> sub_intakeArmPositionBrake.get());
        trg_intakeRollersBrake = new Trigger(() -> sub_intakeRollersBrake.get());
        trg_spindexerBrake = new Trigger(() -> sub_spindexerBrake.get());
        trg_tunnelBrake = new Trigger(() -> sub_tunnelBrake.get());
    }
}
