# Technical Documentation

## Communication between Robot and Webapp
Currently, the codebase is using ROS topics with `from_robot` topic for messages from the Robot and `from_web` topic for messages from the webapp to the robot. This is not the best way of fostering this communication. It would be ideal if we can shift from ROS topics to ROS services.

## States
In [this page](https://github.com/personalrobotics/feeding_web_interface/blob/2022_revamp/feedingwebapp/src/Pages/Constants.js), there are all the states as constants outlined. Each button click is simply calling a function, which eventually calls the `changeState()` function. The `changeState` function takes in a `String` parameter. And this parameter has to be one of the constant values specified in the constants file. The states determines what pages are displayed. For instance, if the app is currently in `Not_Eating` state, then it would be in the first page that gets displayed. But, once the `start feeding` button is clicked, the app progresses to `moving_above_the_plate` state and transitions the app into that state. 

There is essentially a large `if/else` block, which is constantly checking for state changes. As the state changes, a different element gets rendered. Below is a bit of code showing that happening: 
```
else if (currentStateVal.feeding_status == constants.States[2]) {
    return (
        <div style={{ "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
            <h1 className="text-center txt-huge" style={{ "font-size": "40px" }}>Food Item Selection</h1>
            {isConnected ? <div style={{ "display": "block" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "block" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}

            <div style={{ "display": "block" }}><Button className="doneBut" style={{ "font-size": "24px", "margin-top": "0px", "marginRight": "10px", "marginLeft": "auto", "display": "block" }} onClick={() => changeState(constants.States[9])}>✅ Done Eating</Button></div>

            <p class="transmessage" style={{ "margin-bottom": "0px" }}>Choose from one of the following food items.</p>

            <Row xs={3} s={2} md={3} lg={4} className="justify-content-center mx-auto my-2" style={{ paddingBottom: '35vh' }}>
                {food.map((value, i) => (
                    <Button key={i} variant="primary" className="mx-1 mb-1" style={{ paddingLeft: "0px", paddingRight: "0px", marginLeft: "0px", marginRight: "0px", "font-size": "25px" }} value={value} size="lg" onClick={(e) => food_item_clicked(e)}>{value}</Button>
                ))
                }
            </Row>
            <Footer />
        </div>
    );
}
```

#### Note about app states vs. Robot states
App States: 
```
const [message, setMessage] = useState("");
```
This, for instance, is controlling the state of the app. This is internal o the `Home.js` file and would not be accessible to things outside that. In this case, we are using to publish messages we want the user to see. 

Robot States: 
```
const currentStateVal = useStore((state) => state.defaultState);
```
We are using a `useStore` to control the change of states on the app based on changes in states in the robot. So, in the above piece of code, `currentStateVal.feeding_status` is checking for the robot's state as seen by the app. If the value in that is particularly equal to a certain state constant, then we want to execute the code block associated with that. 
