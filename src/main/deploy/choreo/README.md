## Autonomous Paths
This folder is where all the paths that we made for autonomous pathing exist. Such paths were made using [Choreo](https://choreo.autos/), a path making software. Choreo is how we make all of our autonomous paths, and this year, we opted to make seperate paths, and chain them together for our routines.

### How to make your own paths!
If you would like to see what the paths look like for yourself, or even make your own, lucky for you its something thats very simple! First things first, you need to make sure that Choreo is installed([link for convienience](https://github.com/SleipnirGroup/Choreo/releases)), then navigate the menu (**LOCATED IN THE TOP LEFT**) to the *open project* tab. From there, navigate to where this cloned repository exists on *your* computer, and navigate to this folder. Select the ```Rebuilt.chor``` file and you're good to go! You can see the other paths there, and make your own from there as well.


![Choreo Logo](images/choreoLogo.png)

### Example of one of our autons
```java
        addMultiAuton(kLeftTrenchTwoCycleBumpReturn,
            new AdaptableAutonInfo(AutonK.kLeftOneBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, AutonK.kSOTMTimeout, true, 0));
```
Note that we chain 5 paths together to create an auton, in which we run the RIGHT_one_bumpReturn, the RIGHT_two_bumpToTrench, the RIGHT_two_bumpReturn, followed by the RIGHT_two_bumpToTrench, finishing with the RIGHT_two_goOut. All of these paths exist as traj's in this folder, and that is where the code pulls from; such paths are, once again, made in choreo. So in order to make a new auton, you must make a plethora of new paths, and make it so that they chain together using the logic seen above.

### Example of one of our paths (LEFT_one_bumpReturn)
We start at the GREEN waypoint (waypoint one for those of you who are colorblind), and end at the RED waypoint (waypoint 7). The arrow on the square represents the front of our robot, and this year, that is our intake (counter-intuitive, tell me about it :sob:).
![Choreo Path](images/choreoPath.png)
