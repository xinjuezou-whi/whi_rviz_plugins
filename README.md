# whi_rviz_plugins
rviz plugins package for showing custom info. currently only Battery is implemented, which shows the battery's left charge in text

![image](https://user-images.githubusercontent.com/72239958/182014031-264092c1-335d-4b83-a21c-f6fa8b4aefbc.png)

## overview
plugin Battery is a derived class from MessageFilterDisplay, and subscribes message "whi_interfaces::WhiBattery". Here is the definition of such message:

  `std_msgs/Header header\
   uint16 percent\
   bool need_charge`

The base of the charge text will be at the frame listed in the header of the WhiBattery message, which 

## build
`cd <workspace>`
`catkin build`
or catkin_make depends on your environment
  
## use instruction

