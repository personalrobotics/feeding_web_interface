# Technical Documentation

## Communication between Robot and Webapp
Currently, the codebase is using ROS topics with `from_robot` topic for messages from the Robot and `from_web` topic for messages from the webapp to the robot. This is not the best way of fostering this communication. It would be ideal if we can shift from ROS topics to ROS services.

## States
In [this page](https://github.com/personalrobotics/feeding_web_interface/blob/2022_revamp/feedingwebapp/src/Pages/Constants.js), there are all the states as constants outlined. Each button click is simply calling a function, which eventually calls the `changeState()` function. The `changeState` function takes in a `String` parameter. And this parameter has to be one of the constant values specified in the constants file. The states determines what pages are displayed. For instance, if the app is currently in `Not_Eating` state, then it would be in the first page that gets displayed. But, once the `start feeding` button is clicked, the app progresses to `moving_above_the_plate` state and transitions the app into that state. 
