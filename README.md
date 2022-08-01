# whi_rviz_plugins
Rviz plugins package for showing custom info. currently only Battery is implemented, which shows the battery's left charge in text.

![battery](https://user-images.githubusercontent.com/72239958/182051634-f7bd0b34-ea97-4944-b23d-494119465428.gif)

## overview
Plugin Battery is a derived class from MessageFilterDisplay, and subscribes message "whi_interfaces::WhiBattery". Here is the definition of such message:

```
std_msgs/Header header
uint16 percent
bool need_charge
```

The base of the charge text will be at the frame listed in the header of the WhiBattery message, which let the charge info stick to robot and move with it. Besides under multiple robots senario, frame with namespace enable each robot bearing its own charge info.

## build
Clone package `whi_interfaces` and `whi_rviz_plugins` to your workspace:

```
cd <your workspace>
catkin build
```

or `catkin_make` depends on your environment.
  
## use instruction
1. Publish the WhiBattery message;
  For a quick check, there is a test script, named `send_test_msgs.py` under folder `scripts` to publish simulated charge info interatively, just run:

  ```
  python src/whi_rviz_plugins/scripts/send_test_msgs.py 
  ```

2. Run rviz to add the Battery plugin.
  Click the "Add" button at the bottom of the "Displays" panel, then scrolling down through the available displays until you see "Battery" under package name "whi_rviz_plugins"

![image](https://user-images.githubusercontent.com/72239958/182015665-fd271ba8-7fbb-4b4c-b479-73f90e24b48d.png)

  Once the Battery display is added to rviz, set the topic name of the display to a source of "whi_interfaces::WhiBattery". The topic is "test_bat" if you are running the "send_test_msgs.py".
  
![image](https://user-images.githubusercontent.com/72239958/182015784-2942af09-8773-464b-87ce-e31522cf21ec.png)

![image](https://user-images.githubusercontent.com/72239958/182015841-4e2d55bd-2913-41bc-b843-55c7d6f62cf5.png)
