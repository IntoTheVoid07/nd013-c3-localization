# nd013-c3-localization
The third Udacity Self-Driving Car Nanodegree Project was centered around using LiDAR to localize the simulated car using LiDAR scans at regular intervals and a provided known map. The main objective was to localize the simulated car driving for at least 170 meters from the starting position and never exceeding a distance pose error of 1.2 meters. 

NOTE: I'm sure there's a way to build and run locally if you have a Ubuntu Xenial VM. However, getting everything setup for the Udacity projects has been a moot point and a lot of the libraries, etc. seem outdated. So, I opted to just use the provided workspace environment.

## Building the source
NOTE: I had trouble building the source without enabling the GPU capability. So, make sure the GPU is enabled.

From the Udacity workspace desktop, in the one terminal:

```bash
cd /home/workspace/c3-project
cmake .
make
```

## Running the simulator
In a second terminal, launch the simulator by:

```bash
su - student
cd /home/workspace
./run-carla.sh
```

## Running the project code
In the third terminal, run the project's code:

```bash
cd /home/workspace
./cloud_loc
```

Alternatively, I added a rudimentary command line interface where you can configure the following options:

```bash
./cloud_loc <"icp" or "ndt"> <filter resolution, float> <number of iterations, int>
```

The defaults are:

```bash
./cloud_loc "icp" 1.0 10
```


## Controlling the simulated car

To control the car, the PCL has a listener in its viewer window. To move the car, one of the five options can be selected:

| Key | Description |
----------- | ----------- |
| UP arrow | Each tap increases the throttle power by 1 tick |
| DOWN arrow | Each tap will stop the car and reste the throttle to zero, if it's moving. If it's not moving it will apply throttle in the reverse direction |
| LEFT arrow | Incremently changes the steer angle value to the left |
| RIGHT arrow | Incremently changes the steer angle value to the right |
| A | Recenter's the camera with a top down view of the car |
| W | Each tap will increase the throttle power by 3 ticks |

The criteria for this project also specified that the car must be able to continously localize the car when it is moving at a medium speed (3 taps of the up arrow). I had some trouble getting the key presses to work. Therefore, I added an additional key option of "W".
