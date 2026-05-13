# Robot Code

This is the main codebase for Team 2974 (Walton Robotics) for the ReBuilt Season. Typically, everything here is all you'll need, as everything here controls how the robot behaves during a match, from driving around to shooting game pieces at targets.

If you're new to the team, welcome! Hopefully this will help you figure out where things live so you're not completely lost staring at the file tree, like I was ~~two years ago~~ all those years ago.

---

## How the code is organized

The code is split into folders by responsibility. Each folder handles one aspect of the robot. If something breaks, you'll know exactly where to look (and who to blame(thank you gitblame)).

---

## Top-level files

- `Robot.java` — Main robot class. Subsystems, commands, and bindings all get wired up here. Start here for the big picture.
- `Main.java` — Program entry point. You ~~probably~~ never need to touch this.
- `Constants.java` — Robot-wide constants (motor ports, PID values, speed limits, etc.). If you're looking for a magic number, it's probably here. NO MAGIC NUMBERS OUTSIDE OF HERE!!!!
- `FieldConstants.java` — Field dimensions and target positions. All thanks to our goats, Mechanical Advantage (6328)

### [autons/](autons/) — Autonomous routines

Defines what the robot does during the 15-second autonomous period at the start of a match, where no driver input is allowed.
#### this is where we start praying

- `WaltAdaptableAutonFactory.java` — Builds auto routines by chaining Choreo trajectory segments together.

---

### [dashboards/](dashboards/) — Driver station UI

Controls what drivers see on their dashboard laptop.

- `AutonChooser.java` — Dropdown that lets drivers pick which auto routine to run before a match.

---

### [generated/](generated/) — Auto-generated files

Created by external tools, not written by hand. Don't edit these directly unless you know what you're doing LOL

- `TunerConstants.java` — Swerve drive tuning constants from CTRE's Tuner X <small>(THANK GOD FOR TUNERX)</small>.

---

### [subsystems/](subsystems/) — Robot mechanisms

Each file represents a physical mechanism on the robot and the code that controls it. This is where most of the action happens.

#### [subsystems/shooter/](subsystems/shooter/) — Shooting mechanism

The fun part. I had SO much fun on this part :D

- `Shooter.java` — Master control over the entire shooter subsystem.
- `Hood.java` — Adjustable hood angle and other variety.
- `Turret.java` — Turret rotation and tracking.
- `TurretVisualizer.java` — 3D visualization for turret state.
- `FuelSim.java` — Game piece physics simulation for testing.

##### [subsystems/shooter/calc](subsystems/shooter/calc) — Shot calculations

this is where it hit the fan D:

- `ShooterCalc.java` — Shot math and distance-based calculations on its own thread.
- `ShotCalculator.java` — Other shot math and distance-based calculations.

#### Other subsystems

- `Intake.java` — Picks up game pieces from the ground.
- `Indexer.java` — Feeds game pieces from the intake into the shooter. The middleman.
- `Superstructure.java` — Coordinates intake, indexer, and shooter together so nothing fires when it shouldn't.
- `Swerve.java` — Swerve drivetrain. Makes the robot go vroom in any direction.

---

### [vision/](vision/) — Target tracking

Uses cameras to detect and track field targets for automatic aiming. The robot can see better than most of us at this point.

- `WaltCamera.java` — Camera wrapper.
- `Detection.java` — Detects game pieces.
- `VisionSim.java` — Simulates vision for testing.

---