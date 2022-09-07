# Documentation

## How to run the app locally?
- Clone the repo: `git clone git@github.com:personalrobotics/feeding_web_interface.git`
- Then checkout `2022_revamp` branch
- `cd feeding_web_interface/feedingwebapp`
- Perform `npm install` to install all the packages related to this project
- Then, `npm start` to begin the application.

In the `Home.js` file, you can set `debug = true` and run the application in debug mode or set it to `debug = false` and run it along with roscore/ros messages. 

### How to run ROS 'stuff' with the app in `debug = false` mode?
- Make sure you are in a ROS workspace (for the purposes of this document, I will name the workspace created to be `catkin_ws`) with all required ROS packages installed. 
- `cd` into the ROS workspace
- Make sure to open up at least four terminals with all of them inside the workspace. 
- Then, run `source devel/setup.bash` in each of the terminals that are open (Use Tmux to make splitting terminal easier on Ubuntu)
- On the first terminal, run `roscore`
- On the second terminal, run `roslaunch rosbridge_server rosbridge_websocket.launch` to start the webserver. _This will allow the webapp to actually directly connect with the server (when you hit "start feeding" button)._
- On the third terminal (designate this terminal for yourself to be the "ROBOT sending messages to app") - You can publish messages to `/from_robot` topic and communicate with the webapp to make changes to the states. An example message is `rostopic pub /from_robot std_msgs/String "<state>"`. 
- On the fourth terminal (designate this terminal for yourself to be the "ROBOT receiving messages from app") - You can listen to messages from `from_web` topic. An example command is `rostopic echo /from_web` to listen to messages published from the webapp. 

### How to run camera 'stuff' with the app?
The following steps will outline the method to use to run the video collected from a particular topic in the rosbag _These steps might be diffrerent for if you were to connect to Robot's camera (please check)._
- Start by downloading the rosbags that you wish to run and store them in `catkin_ws` workspace mentioned above. 
- Next, create another workspace (_This could potentially also be done without another workspace but might be easier to make a new workspace as it will probably be more organized_). For the purposes of this document, I will name the workspace to be `camera_ws`). 
- Change directory into the `camera_ws`. 
- Make sure to run `source devel/setup.bash`. 
- Then make sure to download the web video sever files by performing the following steps:
  - First, split the terminal and open up two terminals both inside the `camera_ws`. Make sure you perform `source devel/setup.bash`. 
  - In the first terminal, perform the following:  
  ```
  git clone https://github.com/sfalexrog/async_web_server_cpp.git
  cd async_web_server
  git checkout noetic-devel
  ```
  - In the second terminal, perform the following: 
  ```
  git clone https://github.com/RobotWebTools/web_video_server.git
  ```
  Then, perform `catkin build` from one of the terminals. 
  
#### Running Rosbags
- First navigate into your `catkin_ws` workspace and make sure you have your original rosbag files. This means, if you need to decompress, make sure you decompress and store the decompressed rosbag file. Usually, after decompressing, the rosbag file gets stored as <something>.orig.bag. 
- Next, run `rosbag play <name of the bag file>`. 
- You should see the rosbag running its contents. 
- While this is happening, we will need to start the web server to be able to see the images as video. So, we would navigate into the `camera_ws` workspace and change directory into where ever you have stored the `web_video_server`. 
- Inside this folder, run `rosrun web_video_server web_video_server`. 
- After this, you can go to a web browser and type in `localhost:8080` and see that there is a list of topics to choose from. Choose a particular topic to listen to and you should see the video playing. As you do that, you can notice the parameters in web url changing. Make sure you play with the parameters by referring to the document below to get the best quality video for the webapp. 

For further information about this, you can refer to [Web Video server](http://wiki.ros.org/web_video_server) document. 

## What are some next steps? 
- Enabling a method of selecting food from the live video feed that gets displayed in the video tab. 
- What happens if Wifi goes out? 
- What happens if there is accidental refreshes?
- When you first start the webapp, it should first get the status from the robot and update itself to mimic that of the robot. This could be something that can be implemented to eliminate any syncing issues between the robot and the app. Further, in the Settings page, the app should get the settings as default from the robot. 
