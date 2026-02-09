// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.TurretSim.PhysicsSim;
import frc.robot.util.*;

public class Robot extends TimedRobot {
    // Elastic Pub-Subs
    public static NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();
    public static NetworkTable nte_Shooter = nte_inst.getTable("Shooter");

    public static DoubleTopic DT_Shooter = nte_inst.getDoubleTopic("/Shooter/relativeShootingSpeed");
    public static DoublePublisher pub_Shooter;
    public static DoubleSubscriber sub_Shooter;

    public static DoubleTopic DT_Turret = nte_inst.getDoubleTopic("/Shooter/turretPositon");
    public static DoublePublisher pub_Turrert;
    public static DoubleSubscriber sub_Turret;

    // variables
    private Command m_autonomousCommand;
    private final CommandXboxController driver = new CommandXboxController(0);

    // subsystems
    private final Shooter shooter = new Shooter();

    public Robot() {
        initPubSubs();
        configureBindings();
    }

    public void initPubSubs() {
        pub_Shooter = DT_Shooter.publish();
        pub_Shooter.accept(0);
        sub_Shooter = DT_Shooter.subscribe(0.0);

        pub_Turrert = DT_Turret.publish();
        pub_Turrert.accept(0);
        sub_Turret = DT_Turret.subscribe(0);
    }

    public void configureBindings() {
        driver.rightTrigger().whileTrue(shooter.shoot(sub_Shooter));

        driver.rightBumper().onTrue(shooter.setTurretPositionCmd(sub_Turret));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // System.out.println(sub_Shooter.getAsDouble() + "");
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        if (m_autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationInit() {
        // turret.getMotor().getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX44);
        // turret.getMotor().getSimState().setRawRotorPosition(0);
        // PhysicsSim.getInstance().addTalonFX(turret.getMotor(), 0.001);
    }

    @Override
    public void simulationPeriodic() {
        // turret.moveToPosRotations(driver.getLeftY());
        // PhysicsSim.getInstance().run();
        // turret.logData();
    }
}