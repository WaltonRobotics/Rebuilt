import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.BooleanSupplier;

import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.DeployPosition;
import frc.robot.subsystems.Intake.RollersVelocity;

public class IntakeTest {
    private final Intake m_intake = new Intake();
    
    private static final double dDeploy = 0.02;
    private static final double dRollers = 5.0;

    @BeforeEach
    void setup() {
        assert Robot.isSimulation();
    }

    @AfterEach
    void shutdown() {
        m_intake.close();
    }

    @Test
    void deployTest() throws InterruptedException {
        Commands.runOnce(() -> m_intake.setDeployPos(DeployPosition.SAFE));
        Thread.sleep(1000);
        assertEquals(0.2, m_intake.getSimDeployPos().getAsDouble(), dDeploy);
    }

    @Test
    void rollerTest() throws InterruptedException {
        Commands.runOnce(() -> m_intake.setRollersSpeed(RollersVelocity.MAX));
        Thread.sleep(1000);
        assertEquals(50, m_intake.getSimRollersSpeed().getAsDouble(), dRollers);
    }
}
