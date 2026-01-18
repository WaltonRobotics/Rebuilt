package frc.robot.subsystems;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.WaltLogger;
import frc.util.WaltLogger.IntLogger;
import frc.util.WaltLogger.StringLogger;

import static frc.robot.Constants.RobotK.*;

public class Superstructure {

    /* declare subsystems */
    public final EventLoop stateEventLoop = new EventLoop();
    private State m_state = State.IDLE;
    
    
        

        
        
    /* state transitions */
    /* requests */

    /* states */


    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, (() -> m_state == State.IDLE));
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, (() -> m_state == State.INTAKING));
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, (() -> m_state == State.INTOOK));
    public final Trigger stateTrg_shootReady = new Trigger(stateEventLoop, (() -> m_state == State.SHOOT_READY));
    public final Trigger stateTrg_shooting = new Trigger(stateEventLoop, (() -> m_state == State.SHOOTING));
    public final Trigger stateTrg_passReady = new Trigger(stateEventLoop, (() -> m_state == State.PASS_READY));
    public final Trigger stateTrg_passing = new Trigger(stateEventLoop, (() -> m_state == State.PASSING));
    public final Trigger stateTrg_climbing = new Trigger(stateEventLoop, (() -> m_state == State.CLIMBING));
    public final Trigger stateTrg_climbed = new Trigger(stateEventLoop, (() -> m_state == State.CLIMBED));
    public final Trigger stateTrg_releasing = new Trigger(stateEventLoop, (() -> m_state == State.RELEASING));
    public final Trigger stateTrg_released = new Trigger(stateEventLoop, (() -> m_state == State.RELEASED));


    /* configure states/ state transitions */


    /* logs */

    private IntLogger log_stateIdx = WaltLogger.logInt(kLogTab, "state idx");
    private StringLogger log_stateName = WaltLogger.logString(kLogTab, "state name");


    public Superstructure() {

    }




    public enum State {
        IDLE(0, "idle"),
        INTAKING(1, "intaking"),
        INTOOK(2, "intook"),
        SHOOT_READY(3, "shoot ready"),
        SHOOTING(4, "shooting"),
        PASS_READY(5, "pass ready"),
        PASSING(6, "passing"),
        CLIMBING(7, "climbing"),
        CLIMBED(8, "climbed"),
        RELEASING(9, "releasing"),
        RELEASED(10, "released");
        
        public final int idx;
        public final String name;

        private State (int index, String _name) {
            idx = index;
            name = _name;
        }
    }
    
}