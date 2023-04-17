# Feeding Web Interface

This directory contains the web app that users use to interact with the robot-assisted feeding system. This readme includes an overview and contribution guidelines, while [TechDocumentation.md](https://github.com/personalrobotics/feeding_web_interface/blob/2023PreDeployment/feedingwebapp/TechDocumentation.md) includes more specific documentation of the technical aspects of the app.

## Overview
The overall user flow for this robot can be seen below.

![newWebAppWorkflow](https://user-images.githubusercontent.com/26337328/223597500-5e520b7a-eb2b-45ad-b9e8-91fec1bdeba4.jpg)
(Last Updated 2023/03/07)
<!-- ![Web App State Machine](https://user-images.githubusercontent.com/8277986/191333326-c71a1765-475c-40f6-87da-a79b7c73e0ee.png) 
(Last Updated 2022/09/20) -->

## Dependencies
- [`npm`](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm)
<!-- - [ROS](http://wiki.ros.org/noetic/Installation) -->

## Getting Started

### Installation
1. Clone the repo: `git clone git@github.com:personalrobotics/feeding_web_interface.git` using SSH, or `git clone https://github.com/personalrobotics/feeding_web_interface.git` using HTTPS
2. `cd {PATH TO feeding_web_interface}/feedingwebapp`
3. Install all dependencies: `npm install --legacy-peer-deps`

### Usage
1. Start the app: `npm start`. (You may need to run `npm install` before this if the dependencies changed e.g., if you switched to a new branch.)
    1. Note that if you're not running the robot code alongside the app, set [`debug = true` in `App.jsx`](https://github.com/personalrobotics/feeding_web_interface/blob/b8f5d970628bbaac43b8b9c7dbc4349ada2f32d7/feedingwebapp/src/App.jsx#L17) to be able to move past screens where the app is waiting on the robot.
2. Use a web browser to navigate to `localhost:3000` to see the application.

## Contributing

### Documenting Tasks

All tasks should be tracked and documented as [Github Issues](https://github.com/personalrobotics/feeding_web_interface/issues).

### Adding Dependencies
Note that we use `npm`, not `yarn`, to manage dependencies for this project.
- Dependencies added with `npm install ...` should automatically be added when you add `package.json` and `package-lock.json` to your branch.
- Additional dependencies should be documented in this readme.

### Writing Code
- Generally, only work on code if there is a corresponding [Github Issue](https://github.com/personalrobotics/feeding_web_interface/issues) for it.
- Every feature should be on its own branch, and generally only one person should be pushing to one branch.
- Follow the below style guides:
  - [AirBnB React/JSX Style Guide](https://airbnb.io/javascript/react/). This was written before React hooks (e.g, `useState`); see [this for an example of how to order calls to various hooks](https://dev.to/abrahamlawson/react-style-guide-24pp#comment-1f4fd).
  - [AirBnB JavaScript Style Guide](https://airbnb.io/javascript/) for anything not covered in the above guide (e.g., variable naming conventions).
  - [React Styleguidist guide](https://react-styleguidist.js.org/docs/documenting/) for documenting code. Every function and component should be documented, and within functions there should be enough documentation that someone without knowledge of React can understand it.
- Thoroughly test your feature:
  - Run `npm start`, ensure it has no warnings or errors.
  - Thoroughly test your feature, including all edge cases, to ensure it works as expected. This includes trying every combination of buttons/actions, even ones we don't expect users to use, to ensure there are no unaccounted for edge cases.
  - Ensure the [console](https://developer.chrome.com/docs/devtools/console/) has no errors.
  - Thoroughly test responsivity by changing browser/devise size to ensure it renders as expected.
- Before creating a Pull Request, run `npm format` and address any warnings or errors.
- Create a Pull Request to merge your branch into `main`. You need at least one approving review to merge.
- Squash all commits on your branch before merging into `main` to ensure a straightforward commit history.

## Misc Notes (Possibly Outdated)

## Usage
### How to run the app locally?
- Clone the repo: `git clone git@github.com:personalrobotics/feeding_web_interface.git` using SSH, or `git clone https://github.com/personalrobotics/feeding_web_interface.git` using HTTP
- See all branches: `git branch -a` and check to make sure `2022_revamp` branch shows up
- Then checkout `2022_revamp` branch: `git checkout 2022_revamp`
- Run `cd ./feedingwebapp`
- Perform `npm install` to install all the packages related to this project
- Then, `npm start` to begin the application.
- Then, use a web browser to navigate to `localhost:3000` to see the application.

In the `Home.js` file, you can set `debug = true` and run the application in debug mode to experience the GUI of the app without needing it to connect to ROS. Otherwise, set it to `debug = false` and run it along with roscore/ros messages.

#### How to run ROS 'stuff' with the app in `debug = true` mode?
For this, you just need the web app running. And, to mimic the robot/webapp communicating, there are some default buttons throughtout the app that can help mimic the state changes as described in the state machine picture above.

#### How to run ROS 'stuff' with the app in `debug = false` mode?
- `cd` into the ROS workspace
- Make sure to open up at least four terminals with all of them inside the workspace.
- Then, run `source devel/setup.bash` in each of the terminals that are open (Use Tmux to make splitting terminal easier on Ubuntu)
- On the first terminal, run `roscore`
- On the second terminal, run `roslaunch rosbridge_server rosbridge_websocket.launch` to start the webserver. _This will allow the webapp to actually directly connect with the server (when you hit "start feeding" button)._
- On the third terminal (designate this terminal for yourself to be the "ROBOT sending messages to app") - You can publish messages to `/from_robot` topic and communicate with the webapp to make changes to the states. An example message is `rostopic pub /from_robot std_msgs/String "<state>"`.
- On the fourth terminal (designate this terminal for yourself to be the "ROBOT receiving messages from app") - You can listen to messages from `from_web` topic. An example command is `rostopic echo /from_web` to listen to messages published from the webapp.

#### How to run camera 'stuff' with the app?
The following steps will outline the method to use to run the video collected from a particular topic in the [rosbag](http://wiki.ros.org/rosbag) _These steps might be different for if you were to connect to Robot's camera (please check)._
- Start by downloading the rosbags that you wish to run and store them in your ROS workspace.
- Make sure to run `source devel/setup.bash`.
- Then make sure to download the [web video sever](http://wiki.ros.org/web_video_server) files by performing the following steps:
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

##### Running Rosbags
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
- What happens if the user accidentally refreshes?
- When you first start the webapp, it should first get the status from the robot and update itself to mimic that of the robot. This could be something that can be implemented to eliminate any syncing issues between the robot and the app. Further, in the Settings page, the app should get the settings as default from the robot.
- Currently, the E-stop is not accessible when the video modal is open. Consider changing this.
