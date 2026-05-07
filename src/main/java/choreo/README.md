# Choreo

This folder contains our wrapper and utility classes for integrating [Choreo](https://choreo.autos/) into our robot code. Choreo is the path-planning software we use to generate and follow autonomous trajectories.

## Why we forked/duplicated Choreo's library

We duplicated Choreo's library classes locally so we could experiment with **pose correction** during path following. The idea was that if the robot drifts off its expected pose mid-path, we would pause the Choreo timer, drive the robot back to the correct pose, and then resume the timer once the robot is back on track. We weren't able to fully get this working in time, but the groundwork is here.
```java
    if(kUsePoseCorrection) {
      if (poseSupplier.get().getTranslation().getDistance(sample.getPose().getTranslation()) > 0.2) {
          activeTimer.stop();
          outOfBounds.set(true);
      } else if (poseSupplier.get().getTranslation().getDistance(sample.getPose().getTranslation()) < 0.05) {
        activeTimer.start();
        outOfBounds.set(false);
      }
    }
```

## Structure

### auto/
Handles autonomous routine selection and execution.

- `AutoChooser.java` - lets drivers select an auto routine from the dashboard
- `AutoFactory.java` - constructs auto routines by chaining trajectory segments together
- `AutoRoutine.java` - represents a full autonomous routine made up of multiple trajectories
- `AutoTrajectory.java` - wraps a single Choreo trajectory for command-based execution

### trajectory/
Core trajectory representation and sampling logic.

- `Trajectory.java` - the main trajectory class, holds a list of samples and handles time-based lookups
- `TrajectorySample.java` - a single sampled state along a trajectory (pose, velocity, etc.)
- `SwerveSample.java` - trajectory sample specific to swerve drivetrains
- `DifferentialSample.java` - trajectory sample specific to differential drivetrains
- `EventMarker.java` - represents a timed event trigger within a trajectory

### util/
Utility and configuration classes.

- `Choreo.java` - main entry point for loading trajectories from deploy files
- `ChoreoAlert.java` - handles driver station alerts related to Choreo
- `ChoreoAllianceFlipUtil.java` - flips trajectories for red/blue alliance mirroring
- `ChoreoArrayUtil.java` - array helper methods used internally
- `FieldDimensions.java` - field size constants used for alliance flipping
- `TrajSchemaVersion.java` - version tracking for the trajectory file format