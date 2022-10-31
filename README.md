# whi_rviz_plugins
Rviz plugins package for showing custom info. currently there are two plugins: one is the Battery which shows the battery's left charge in text and symbolizing battery, and another is Navi_waypoints which allows user add multiple navigation targets with interactive markers.

- [Battery](#battery)
- [Navi_waypoints](#navi_waypoints)
- [build](#build)
- [use Battery](#use-battery)
- [use Navi_waypoints](#use-navi_waypoints)

## Battery
Plugin Battery is a derived class from MessageFilterDisplay:

![battery](https://user-images.githubusercontent.com/72239958/187207845-3ec2cd87-ec7e-437b-9cd5-7884a6faf08e.gif)

It subscribes message "whi_interfaces::WhiBattery". Here is the definition of such message:

```
std_msgs/Header header
uint16 percent
bool need_charge
```

The base of the charge text will be at the frame listed in the header of the WhiBattery message, which let the charge info stick to robot and move with it. Besides under multiple robots senario, frame with namespace enable each robot bearing its own charge info.

## Navi_waypoints
Plugin Navi_waypoints derives from class rviz::Display, and creates a panel for waypoints interaction logic:

![waypoints_add](https://user-images.githubusercontent.com/72239958/198868281-a7e562d9-c85d-4ec5-ab89-87e6f34de6eb.gif)


## build
Clone package `whi_interfaces` and `whi_rviz_plugins` to your workspace:

```
cd <your_workspace>
catkin build
```

or `catkin_make` depends on your environment. And don't forget to source the bash:
```
source <your_workspace>/devel/setup.bash
```

## use battery
1. Publish the WhiBattery message

  For a quick check, there is a test script, named `send_test_msgs.py` under folder `scripts` to publish simulated charge info interatively. Before running it, please make sure roscore is active:
  ```
  roscore
  ```
  and open a new terminal and run:
  ```
  python src/whi_rviz_plugins/scripts/send_test_msgs.py 
  ```

2. Run rviz to add the Battery plugin

  Click the "Add" button at the bottom of the "Displays" panel, then scrolling down through the available displays until you see "Battery" under package name "whi_rviz_plugins"

![image](https://user-images.githubusercontent.com/72239958/182015665-fd271ba8-7fbb-4b4c-b479-73f90e24b48d.png)

  Once the Battery display is added to rviz, set the topic name of the display to a source of "whi_interfaces::WhiBattery". The topic is "test_bat" if you are running the "send_test_msgs.py".
  
![image](https://user-images.githubusercontent.com/72239958/182015784-2942af09-8773-464b-87ce-e31522cf21ec.png)

![image](https://user-images.githubusercontent.com/72239958/187208878-81e3d48a-a54f-4eb4-af29-1ef49cd71409.png)

### Local position

Modify the offsets to the center of frame to adjust its local position:

![local](https://user-images.githubusercontent.com/72239958/187211993-2fa4fc94-10c3-432e-85df-b959d0695dc2.gif)

### Local orientation

Modify the orientation to the frame to adjust the direction of battery symbol, this is helpfull in navigation senario which is 2D view of XY plane:

![orientaion](https://user-images.githubusercontent.com/72239958/187387702-172637b1-0e06-4356-9259-c1d16afceebc.gif)

![image](https://user-images.githubusercontent.com/72239958/187388565-0340d940-fc04-4a1f-aca6-e193c1c8feaa.png)

> Navigation robot should have a static TF link that directs to map. The belowing gif shows the static TF in robot's URDF(battery->base_link->map)

![bat](https://user-images.githubusercontent.com/72239958/197972731-3d453537-44c4-4a22-9038-617c21d2711d.gif)

## use Navi_waypoints
1. Add the Navi_waypoints plugin to RViz
Click the "Add" button at the bottom of the "Displays" panel, then scrolling down through the available displays until you see "Navi_waypoints" under package name "whi_rviz_plugins":
![image](https://user-images.githubusercontent.com/72239958/198869331-34ce45d3-8879-4535-a3ff-5046b597dca4.png)

2. Add waypoints and adjust its position and orientation through interfactive marker
![waypoints_add](https://user-images.githubusercontent.com/72239958/198868281-a7e562d9-c85d-4ec5-ab89-87e6f34de6eb.gif)

> Remember to switch the mode from "Move Camera" to "Interact" to grand the accessiblity of waypoint marker


3. Click "Execute" to start the multiple goal's navigation
![waypoints_execute](https://user-images.githubusercontent.com/72239958/198869441-af36d932-8744-413e-907e-957bf3ac91ec.gif)

### Marker Height Properties

Since the view orientation is top-down in navigation, multiple info are projected on map and overlay each other. If the interactive marker is happen to be overlaid by other info, it could not be accessed by user. Therefore the property marker height is introduced to make sure it is on the top of other info, so to be accessible.

It is suggested to set its height over the max of your robot, default is 1 meter

### Point/Stop Span params

These two params on Navi_waypoints panel are used to set the duration between two adjacent goals. If the execution is set as loop mode, the stop span stands for the duration between the last goal and the first goal.

The duration is measured as time, so these two's unit are second. Both are within range -10800 to 10800. Negative duration means there is no stop between goals, and the current goal will be preempted x seconds ahead its arrival by the next goal. The positive means the stop duration between goals.

### Save and Load

For the convenience, you can save the waypoints by clicking save button, and re-use them by button load. The waypoints file is saved as yaml format.

