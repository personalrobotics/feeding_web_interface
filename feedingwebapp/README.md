# Documentation

## How to run the app locally?
- Clone the repo: `git clone git@github.com:personalrobotics/feeding_web_interface.git`
- Then checkout `2022_revamp` branch
- `cd feeding_web_interface/feedingwebapp`
- Perform `npm install` to install all the packages related to this project
- Then, `npm start` to begin the application.

In the `Home.js` file, you can set `debug = true` and run the application in debug mode or set it to `debug = false` and run it along with roscore/ros messages. 

### How to run ROS 'stuff' with the app in `debug = false` mode?
- Make sure you are in a ROS workspace with all required ROS packages installed. 
- `cd` into the ROS workspace
- Make sure to open up at least four terminals with all of them inside the workspace. 
- Then, run `source devel/setup.bash` in each of the terminals that are open (Use Tmux to make splitting terminal easier on Ubuntu)
- On the first terminal, run `roscore`
- On the second terminal, run `roslaunch rosbridge_server rosbridge_websocket.launch` to start the webserver. _This will allow the webapp to actually directly connect with the server (when you hit "start feeding" button)._
- On the third terminal (designate this terminal for yourself to be the "ROBOT sending messages to app") - You can publish messages to `/from_robot` topic and communicate with the webapp to make changes to the states. An example message is `rostopic pub /from_robot std_msgs/String "<state>"`. 
- On the fourth terminal (designate this terminal for yourself to be the "ROBOT receiving messages from app") - You can listen to messages from `from_web` topic. An example command is `rostopic echo /from_web` to listen to messages published from the webapp. 

### How to run camera 'stuff' with the app?
The following steps will outline the method to use to run the video collected from a particular topic in the rosbag _These steps might be diffrerent for if you were to connect to Robot's camera (please check)._
- First, 

## What are some next steps? 
- Enabling a method of selecting food from the live video feed that gets displayed in the video tab. 
- What happens if Wifi goes out? 
- What happens if there is accidental refreshes?
- When you first start the webapp, it should first get the status from the robot and update itself to mimic that of the robot. This could be something that can be implemented to eliminate any syncing issues between the robot and the app. Further, in the Settings page, the app should get the settings as default from the robot. 
