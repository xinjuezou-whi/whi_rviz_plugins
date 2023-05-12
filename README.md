# whi_rviz_plugins
Rviz plugins package for showing custom info. currently there are five plugins: the Battery which shows the battery's left charge in text and symbolizing battery; the Navi_waypoints which allows user add multiple navigation targets with interactive markers; the Teleop which publishes the twist message through GUI; and the panel Map_saver which helps user to save map directly by RViz without typing command in terminal; the Video_stream which derived from default image plugin with extended stream sources like webcam, IP stream, and video file.

- [Battery](#battery)
- [Navi_waypoints](#navi_waypoints)
- [Teleop](#teleop)
- [Map_saver](#map_saver)
- [Video_stream](#video_stream)
- [Navi_namespace](#navi_namespace)
- [Build](#build)
- [Use Battery](#use-battery)
- [Use Navi_waypoints](#use-navi_waypoints)
- [Use Teleop](#use-teleop)
- [Use Map_saver](#use-map_saver)
- [Use Video_stream](#use-video_stream)
- [Use Navi_namespace](#use-navi_namespace)

## Battery
Plugin Battery is a derived class from MessageFilterDisplay:

![battery](https://user-images.githubusercontent.com/72239958/187207845-3ec2cd87-ec7e-437b-9cd5-7884a6faf08e.gif)

It subscribes message "whi_interfaces::WhiBattery". Here is the definition of such message:

```
std_msgs/Header header
uint16 soc
uint8 STA_NORMAL=0
uint8 STA_CHARGING=1
uint8 STA_NEED_CHARGING=2
int8 state
```

The base of the charge text will be at the frame listed in the header of the WhiBattery message, which let the charge info stick to robot and move with it. Besides under multiple robots senario, frame with namespace enable each robot bearing its own charge info.

## Navi_waypoints
Plugin Navi_waypoints derives from class rviz::Display, and creates a panel for waypoints interaction logic:
![waypoints_im](https://user-images.githubusercontent.com/72239958/198922851-85c9cbee-87e3-4eca-871b-9a39282a0e05.gif)

From version 00.09 it supports the namespace for manipulating multiple robots simultaneously:
![multiple_waypoints](https://user-images.githubusercontent.com/72239958/233375738-db73c3f4-9613-462e-8808-ec631c5aef5d.gif)

## Teleop
Teleop's function is same as the one operated through terminal but with graphic interaction which can be more convenient in senario like mapping. User can navigate the robot directly through the mapping's RViz window without opening another terminal:
![teleop](https://user-images.githubusercontent.com/72239958/202849722-3b94f105-314e-4de9-b6fb-4c0973ace9c4.gif)

## Map_saver
Map_saver is a panel type plugin. It can help user to save map directly through the mapping's RViz window without typing save commands in another terminal:
![mapsaver](https://user-images.githubusercontent.com/72239958/202850327-21740d9a-5339-45bb-a772-2a1c1e2f22fc.gif)

## Video_stream
This plugin derived from the default image plugin with extended stream sources including webcam, IP stream, and video file:
![stream](https://user-images.githubusercontent.com/72239958/204996337-f5bc2fae-bb17-4306-b71b-8beb10904a62.gif)

## Navi_namespace
This plugin dervied from the PoseTool, and extended to functionalities with both 2D Pose Estimate and 2D Nav goal. With this plugin user can navigate specified robot by GUI rather than laborious texting by Tool Properties:
![multiple](https://user-images.githubusercontent.com/72239958/232202523-065316e1-c1eb-497a-9a42-6731e7f24d40.gif)

## Build
Clone package `whi_interfaces` and `whi_rviz_plugins` to your workspace:

```
cd <your_workspace>/src
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
git clone https://github.com/xinjuezou-whi/whi_rviz_plugins.git
cd ..
catkin build
```

or `catkin_make` depends on your environment. And don't forget to source the bash:
```
source <your_workspace>/devel/setup.bash
```

## Use battery
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

  Click the "Add" button at the bottom of the "Displays" panel, then scrolling down through the available displays until you see "Battery" under package name "whi_rviz_plugins":

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

## Use Navi_waypoints
1. Add the Navi_waypoints plugin to RViz

Click the "Add" button at the bottom of the "Displays" panel, then scrolling down through the available displays until you see "Navi_waypoints" under package name "whi_rviz_plugins":

![image](https://user-images.githubusercontent.com/72239958/198869331-34ce45d3-8879-4535-a3ff-5046b597dca4.png)

2. Add waypoints and adjust its position and orientation through interfactive marker

![waypoints_add](https://user-images.githubusercontent.com/72239958/198868281-a7e562d9-c85d-4ec5-ab89-87e6f34de6eb.gif)

> Remember to switch the mode from "Move Camera" to "Interact" to grand the accessiblity of waypoint marker


3. Click "Execute" to start the multiple goal's navigation

![waypoints_execute](https://user-images.githubusercontent.com/72239958/198869441-af36d932-8744-413e-907e-957bf3ac91ec.gif)


### Marker Height property

Since the view orientation is top-down in navigation, multiple info are projected on map and overlay each other. If the interactive marker is happen to be overlaid by other info, it could not be accessed by user. Therefore the property marker height is introduced to make sure it is on the top of other info, so to be accessible.

It is suggested to set its height over the max of your robot, default is 1 meter

### Point/Stop Span params

These two params on Navi_waypoints panel are used to set the duration between two adjacent goals. If the execution is set as loop mode, the stop span stands for the duration between the last goal and the first goal.

The duration is measured as time, so these two's unit are second. Both are within range -10800 to 10800. Negative duration means there is no stop between goals, and the current goal will be preempted x seconds ahead its arrival by the next goal. The positive means the stop duration between goals.

### Save and Load

For the convenience, you can save the waypoints by clicking save button, and re-use them by button load. The waypoints file is saved as yaml format.

### Namespace

Under multiple robots senario, using the namespace drop list to set the namespace for specified robot.

## Use Teleop
1. Add the Navi_waypoints plugin to RViz

Click the "Add" button at the bottom of the "Displays" panel, then scrolling down through the available displays until you see "Teleop" under package name "whi_rviz_plugins":

![image](https://user-images.githubusercontent.com/72239958/202850781-d94e9f6d-91d2-4520-a68a-a1fc409d4675.png)

2. click direction buttons or press keys to navigate the robot

keys definition

![Untitled Diagram drawio](https://user-images.githubusercontent.com/72239958/202851886-e404eafc-dae1-488b-bbb4-356eb4cca441.png)

> Keys are functional while the "Key" indicator is light on in green. If it is gray click the anywhere of the teleop panel to activate it.
> ![activate](https://user-images.githubusercontent.com/72239958/202852068-e9f2c921-3cc8-46ae-880f-5bece0c381a1.gif)

### Enable Teleop property
This property toggles the publish of twist message. It is helpfull to avoid the collsion of cmd_vel while the plugin is added in navigation's configuration.

## Use Map_saver
Add the Teleop panel by opening the "Panels" menu and then "Add New Panel" within that. This should bring up a Panel class chooser dialog, and select the "Map_saver" within whi_rviz_plugins":

![image](https://user-images.githubusercontent.com/72239958/202853152-601e29cf-00cf-4d0b-8c8f-170ad75a8948.png)

Once the map is satified to you, just click the "Save" button to save it:

![image](https://user-images.githubusercontent.com/72239958/202853295-8db92ae3-2eec-4a9a-bad7-8a09b027c21c.png)

## Use Video_stream
1. Add the Video_stream plugin to RViz

Click the "Add" button at the bottom of the "Displays" panel, then scrolling down through the available displays until you see "Video_stream" under package name "whi_rviz_plugins":

![image](https://user-images.githubusercontent.com/72239958/204996785-4818279d-578f-4d36-83e2-96a58c5bc580.png)

2. Specify the stream source

| Source  | Type                          | Example                                                                                   |
|---------|-------------------------------|-------------------------------------------------------------------------------------------|
| Message | sensor_msgs::Image            | ROS default image message                                                                 |
| Device  | webcam                        | /dev/video1, just input the index of video device, like 1 in this example                 |
| URL     | IP video stream or video file | rtsp://xxxx or http://xxxx or https://xxxx and even a video file like /home/user/xxxx.mp4 |

## Use Navi_namespace
1. Add the Navi_namespace plugin to RViz

Click the "+" icon at the tool bar, then scrolling down through the available tools until your see "Navi_namespace" under package name "whi_rviz_plugins":

![image](https://user-images.githubusercontent.com/72239958/232203203-955d5f4c-b5d8-47fa-9cee-de7cb9702ca8.png)

2. Add namespace

Input namespace in combox, then click button "Add". If namespace is added successfully, the Navi_namespace panel will show the total count of added namespaces:

![image](https://user-images.githubusercontent.com/72239958/232203413-5bad7ee2-7f95-415a-8ab5-15c9fe91170f.png)
![image](https://user-images.githubusercontent.com/72239958/232203575-be620bb2-596e-4cce-a9c2-6084ba0a2977.png)

3. Toggle between Initial Pose and Navi Goal for setting initial pose and sending out navigation goal:

![toggle](https://user-images.githubusercontent.com/72239958/232205172-eebd5188-f570-41bc-87d2-c040ff8fd5a3.gif)

> NOTE: In a multi-machine operating environment, the timestamps between different machines need to be synchronized. Otherwise, there may be situations where executing navigation commands fails, as shown in the following example: the left image is the timestamp of the master, and the right image is the timestamp of the client. The master time is about 2 minutes ahead of the client time:
> ![timestamp](https://github.com/xinjuezou-whi/whi_rviz_plugins/assets/72239958/85a836ac-cc98-4421-8dfd-2579fb014941)
> 
> When using the Navi_waypoints plugin in the master to perform multi-point path navigation operations, the current target timestamp recorded in the robot's actionlib is the current time of the master. If a navigation target operation is sent again using 2D Navi Goal (whether from the master or the client itself), navigation cannot be performed. This is because the Weihang Intelligence plugin Navi_waypoints uses the simple action client to send navigation targets, and the timestamp used by this method is the master's own time. When actionlib receives the target, it will assign this target as its internal current target. On the other hand, 2D Navi Goal uses the message method with the topic move_base_simple/goal to send navigation targets. Under this method, when move_base receives the target message (type geometry_msgs::PoseStamped), it converts the target into move_base's target message move_base_msgs::MoveBaseActionGoal, and assigns its own time to it before forwarding the message to actionlib. Therefore, there may be a situation where the timestamp of the new navigation target is earlier than the current target timestamp recorded by actionlib. In this case, actionlib will consider it an invalid target and abandon its execution.
> 
> There are already plenty of well-established methods for timestamp synchronization, among them we recommend NTP. Please google it for setting up
