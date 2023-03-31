import React, { useState, useEffect, useRef } from 'react';
import './Home.css'
import Col from 'react-bootstrap/Col'
import { useROS } from 'react-ros'
import ScriptTag from 'react-script-tag';

import { toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

import Button from 'react-bootstrap/Button';
import Row from 'react-bootstrap/Row';
import useStore from "../useStore";
import * as constants from '../Constants';
import Footer from "../Footer/Footer";
import ROSLIB from "roslib";


const debug = true;
let food = ["Apple", "Banana", "Carrot", "Cucumber", "Lettuce", "Mango", "Orange", "Pumpkin"];

var listener = [];
function Home() {
    // Calls the useROS() to get instantiation and returns the following variables
    let { ros, isConnected, topics, services, toggleConnection, createListener } = useROS();

    // Topic to get message from the robot
    var new_topic_from_Robot = {
        path: "/from_robot",
        msgType: "std_msgs/String",
        type: "topic"
    }

    // Add this topic to the list of topics provided from useROS()
    topics[4] = new_topic_from_Robot;
    console.log(topics);

    const [topic, setTopic] = useState('/');
    const [queue, setQueue] = useState(0);
    const [message, setMessage] = useState("");
    const [compression, setCompression] = useState('none');

    // Topic to send message from webapp to robot
    const [fromWebAppTopic, setFromWebAppTopic] = useState(new ROSLIB.Topic({
        ros: ros,
        name: '/from_web',
        messageType: 'std_msgs/String'
    }));

    useEffect(() => {
        handleTopic(topic, callbackFn);
        runConnection();
    });

    const unsubscribe = () => {
        if (listener) {
            console.log("Unsubscribing");
            for (let i = 0; i < listener.length; i++) {
                console.log(listener[i])
                listener[i].unsubscribe();
            }
        }
    }

    // Some functions that handles topics from roslibjs
    const handleTopic = (topicInput, callbackFn) => {
        unsubscribe();
        for (var i in topics) {
            if (topics[i].path == topicInput) {
                let j = listener.length - 1;
                console.log(topics[i]);
                listener[j] = createListener(topics[i].path,
                    topics[i].msgType,
                    Number(queue),
                    compression);
                if (listener) {
                    console.log("Subscribing to messages...");
                    listener[j].subscribe(callbackFn);
                } else {
                    console.log("Topic '" + topic + "' not found...make sure to input the full topic path - including the leading '/'");
                }
                console.log(listener);
                j += 1;
                break;
            }
        }

    }

    const handleTopics = () => {
        handleTopic("/from_robot", callbackFn);
    }

    const handleQueue = (queueInput) => {
        setQueue(queueInput);
    }
    const handleCompression = (compInput) => {
        setCompression(compInput);
    }
    const callbackFn = (msg) => {
        console.log(msg);
        let mssg = msg["data"].split("; ");
        if (mssg.length > 1) {
            setMessage(mssg[0]);
            console.log(mssg[1])
            let foodList = [];
            let foodTemp = mssg[1].split(", ")
            for (let i = 0; i < foodTemp.length; i++) {
                foodList.push(foodTemp[i]);
            }
            food = foodList;
        } else {
            setMessage(msg["data"]);
            food = ["Apple", "Banana", "Carrot", "Cucumber", "Lettuce", "Mango", "Orange", "Pumpkin"];
        }
        console.log(message)
    }

    const currentStateVal = useStore((state) => state.defaultState);
    console.log(currentStateVal);
    const changeState = useStore((state) => state.changeState);
    const [foodTrying, setFoodTrying] = useState("");

    // Notification/Alert definition
    const notifyTimeout = () => toast("We are having trouble connecting to the robot. Please refresh the page and try again.");

    function getWidthHeight(imageWidth = 80, imageHeight = 40) {
        let imageAspectRatio = imageWidth / imageHeight;
        let windowWidth = window.innerWidth;
        let windowHeight = window.innerHeight;
        console.log(windowWidth, windowHeight)
        let returnWidth = 0;
        let returnHeight = 0;
        if (windowWidth / windowHeight > imageAspectRatio) {
            returnHeight = windowHeight - 70;
            returnWidth = imageAspectRatio * returnHeight;
        } else {
            returnWidth = windowWidth;
            returnHeight = (1 / imageAspectRatio) * returnWidth;
        }
        console.log(returnWidth)
        console.log(returnHeight)
        return {
            "width": returnWidth,
            "height": returnHeight
        }
    }

    function runConnection() {
        if (!isConnected) {
            toggleConnection();
        }
        handleTopics();
    }

    // Functions that change states
    function start_feeding_clicked() {
        console.log("start_feeding_clicked");
        runConnection();
        fromWebAppTopic.publish(new ROSLIB.Message({
            data: "start_feeding_clicked"
        }));
        if (isConnected || debug) {
            console.log("debug ADAthon")
            changeState(constants.States[10]);
            // changeState(constants.States[1]);
        } else {
            notifyTimeout();
        }
    }

    function exitPlateLocator() {
        console.log("Exit Plate Locator");
        // State 1: "Moving above the plate"
        changeState(constants.States[1]);
    }

    // Temp Button #1
    function moving_above_plate_done() {
        console.log("moving_above_plate_done");
        // State 2: "Feeding"
        changeState(constants.States[2]);
    }

    function food_item_clicked(e) {
        console.log("food_item_clicked" + e.target.value);
        // State 3: "Acquiring Food Item"
        let strData = 'food_item_clicked: ';
        strData += e.target.value;
        fromWebAppTopic.publish(new ROSLIB.Message({
            data: strData
        }));
        changeState(constants.States[3]);
    }

    // Temp Button #2
    function acquiring_food_item_done() {
        console.log("acquiring_food_item_done");
        // State 11: "Status_of_food_on_fork"
        changeState(constants.States[11]);
    }

    function food_retrying() {
        console.log("food_retrying: " + foodTrying);
        let strData = 'food_item_clicked: ' + foodTrying;
        fromWebAppTopic.publish(new ROSLIB.Message({
            data: strData
        }));
        changeState(constants.States[3]);
    }

    function acquiring_food_successful() {
        console.log("acquiring_food_successful");
        // State 4: "Waiting for user to open mouth"
        changeState(constants.States[4]);
    }

    function ready_for_bite_clicked() {
        console.log("ready_for_bite_clicked");
        // State 5: "Moving closer to your mouth"
        fromWebAppTopic.publish(new ROSLIB.Message({
            data: "ready_for_bite_clicked"
        }));
        changeState(constants.States[5]);
    }

    // Temp Button #3
    function moving_closer_mouth_done() {
        console.log("moving_closer_mouth_done");
        // State 6: "Waiting for user to complete the bite"
        changeState(constants.States[6]);
    }

    function done_with_bite_clicked() {
        console.log("done_with_bite_clicked");
        // State 1: "Moving above the plate"
        fromWebAppTopic.publish(new ROSLIB.Message({
            data: "done_with_bite_clicked"
        }));
        changeState(constants.States[1]);
    }

    function done_with_arm_stowing() {
        console.log("done_with_arm_stowing");
        // State 7: "Not Eating"
        fromWebAppTopic.publish(new ROSLIB.Message({
            data: "done_with_arm_stowing"
        }));
        changeState(constants.States[7]);
        if (isConnected) {
            toggleConnection();
        }
    }

    // Connection function for sending messages to the robot (using regular JS)
    function RosConnect() {
        console.log("success")
        var ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' })
        ros.on('connection', function () {
            console.log('Connected to websocket server.');
        });

        ros.on('error', function (error) {
            console.log('Error connecting to websocket server: ', error);
        });

        ros.on('close', function () {
            console.log('Connection to websocket server closed.');
        });

        var fromWebTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/from_web',
            messageType: 'std_msgs/String'
        });

        setFromWebAppTopic(fromWebTopic);
    }

    // Rendering based on current statuses

    // State 7: Not Eating & State 8: Emergency Termination
    if (currentStateVal.feeding_status == constants.States[7] || currentStateVal.feeding_status == constants.States[8]) {
        return (
            <div >
                <div style={{ "display": "block", "width": "100%", "height": "120vh", "overflow-x": "hidden", "overflow-y": "scroll" }} className="outer">
                    <ScriptTag type="text/javascript" src="http://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js" />
                    <ScriptTag type="text/javascript" src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js" />
                    {RosConnect}

                    <h1 className="text-center txt-huge" style={{ "font-size": "40px" }}>🏠 Home</h1>
                    {isConnected ? <div><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}

                    <Row xs={1} md={1} className="justify-content-center mx-2 my-2" >
                        <p class="transmessage" style={{ "margin-bottom": "10px", "margin-top": "0px", "font-size": "24px" }}>Hello!👋 I am ADA's faithful assistant, ADAWebapp! Bon Appétit! 😋</p>
                        <Button variant="primary" size="lg" className="btn-huge" id="#startBtn"
                            onClick={start_feeding_clicked} style={{ width: "75%", "font-size": "35px", "margin-top": "30px" }}>Start Feeding</Button>
                    </Row>
                    <Footer />
                </div>
            </div>
        )
    }

    // State 10: "Plate Locator"
    else if (currentStateVal.feeding_status == constants.States[10]) {
        return (
            <div style={{ "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                {isConnected ? <div style={{ "display": "inline-block" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "inline-block" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}
                <h1 className="text-center txt-huge" style={{ "font-size": "40px" }}>🍽️ Plate Locator</h1>
                <iframe src={`http://localhost:8080/stream?topic=/camera/color/image_raw&default_transport=compressed&width=${Math.round(getWidthHeight().width) - 30}&height=${Math.round(getWidthHeight().height)}&quality=20`}
                    frameborder='0'
                    allow='autoplay; encrypted-media'
                    allowfullscreen
                    title='video'
                    style={{ "width": getWidthHeight().width, "height": getWidthHeight().height }}
                />
                <div class="container">
                    <div class="row">
                        <div class="col">
                            <Button variant="primary" id="tbg-btn-1" style={{ "font-size": "25px", "margin-left": '15%'}} >
                                Forward
                            </Button>
                        </div>

                        <div class="col">
                            <Button variant="success" id="tbg-btn-2" onClick={exitPlateLocator} style={{width:"96%", "font-size": "25px", "margin-bottom": "5%", "margin-left": '10%', "margin-right": "2%"}}>
                                Looks Good
                            </Button>
                        </div>

                        <div class="w-100"></div>

                        <div class="col-3">
                            <Button variant="primary" id="tbg-btn-3" style={{ "font-size": "25px", "margin-left": '2%' }}>
                                Left
                            </Button>
                        </div>

                        <div class="col">
                            <Button variant="primary" id="tbg-btn-4" style={{ "font-size": "25px", "margin-left": '3%' }}>
                                Right
                            </Button>
                        </div>
 
                        <div class="w-100"></div>
                        <div class="col">
                            <Button variant="primary" id="tbg-btn-5" style={{ "font-size": "25px", "margin-left": '15%', "margin-top": "5%", "margin-right": '15%' }}>
                                Backward
                            </Button>
                        </div>
                        <div class="col">
                            <Button id="tbg-btn-6" variant="warning" style={{width:"96%", "font-size": "25px", "margin-left": '10%', "margin-right": "2%" }}>
                                Center Plate
                            </Button>
                        </div>
                    </div>
                </div>
            </div>
        )
    }

    // State 1: "Moving above the plate"
    else if (currentStateVal.feeding_status == constants.States[1]) {
        return (
            <div style={{ "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                {isConnected ? <div style={{ "display": "inline-block" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "inline-block" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}
                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ?
                        <div>
                            <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={moving_above_plate_done}>{constants.States[1]}_DONE</Button>
                            {message == "moving_above_plate_done"
                                ? moving_above_plate_done()
                                : <div>
                                    <h1 id={constants.States[1]} className="waitingMsg">Waiting for Robot to move above the plate...</h1>
                                </div>}
                        </div>
                        : message == "moving_above_plate_done"
                            ? moving_above_plate_done()
                            : <div>
                                <h1 id={constants.States[1]} className="waitingMsg">Waiting for Robot to move above the plate...</h1>
                            </div>
                    }
                </Row>
                <Footer />
            </div>
        )
    }
    // State 2: "Feeding"
    else if (currentStateVal.feeding_status == constants.States[2]) {
        return (
            <div style={{ "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                <h1 className="text-center txt-huge" style={{ "font-size": "40px" }}>Food Item Selection</h1>
                {isConnected ? <div style={{ "display": "block" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "block" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}

                <div style={{ "display": "block" }}>
                    <Button className="doneBut" style={{ "font-size": "24px", "margin-top": "0px", "marginRight": "10px", "marginLeft": "auto", "display": "block" }} onClick={() => changeState(constants.States[10])}>🍽️ Locate Plate</Button>
                    <Button className="doneBut" style={{ "font-size": "24px", "margin-top": "0px", "marginRight": "10px", "marginLeft": "auto", "display": "block" }} onClick={() => changeState(constants.States[9])}>✅ Done Eating</Button>
                </div>

                <p class="transmessage" style={{ "margin-bottom": "0px" }}>Choose from one of the following food items.</p>

                <Row xs={3} s={2} md={3} lg={4} className="justify-content-center mx-auto my-2" style={{ paddingBottom: '35vh' }}>
                    {food.map((value, i) => (
                        <Button key={i} variant="primary" className="mx-1 mb-1" style={{ paddingLeft: "0px", paddingRight: "0px", marginLeft: "0px", marginRight: "0px", "font-size": "25px" }} value={value} size="lg" onClick={(e) => { setFoodTrying(e); food_item_clicked(e) }}>{value}</Button>
                    ))
                    }
                </Row>
                <Footer />
            </div>
        );
    }
    // State 3: "Acquiring Food Item"
    else if (currentStateVal.feeding_status == constants.States[3]) {
        return (
            <div style={{ "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                {isConnected ? <div style={{ "display": "inline-block" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "inline-block" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}

                <div style={{ "display": "inline-block" }}><Button className="cancelBut" style={{ "font-size": "24px", "margin-top": "0px" }} onClick={() => changeState(constants.States[1])}>🗑 Cancel Bite</Button></div>

                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ?
                        <div>
                            <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={acquiring_food_item_done}>{constants.States[3]}_DONE</Button>
                            {
                                message == "acquiring_food_item_done"
                                    ? acquiring_food_item_done()
                                    : <div>
                                        <h1 id={constants.States[3]} className="waitingMsg">Waiting for Robot to acquire the food item selected...</h1>
                                    </div>
                            }
                        </div>
                        : message == "acquiring_food_item_done"
                            ? acquiring_food_item_done()
                            : <div>
                                <h1 id={constants.States[3]} className="waitingMsg">Waiting for Robot to acquire the food item selected...</h1>
                            </div>
                    }
                </Row>
                <Footer />
            </div>
        )
    }

    // State 11: "Status of food on fork"
    else if (currentStateVal.feeding_status == constants.States[11]) {
        return (
            <div style={{ "display": "block", "width": "100%", "height": "115vh", "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                <h1 className=" text-center txt-huge" style={{ "font-size": "40px" }}>Status of Food on fork</h1>
                {isConnected ? <div style={{ "display": "inline" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "inline" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}
                <p class="transmessage" style={{ "margin-bottom": "0px" }}>Click the below button to indicate whether food is on fork.</p>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="primary" className="mx-2 mb-2 btn-huge" size="lg" onClick={acquiring_food_successful} style={{ width: "75%", "font-size": "35px" }}>Fork has food!</Button>
                    <Button variant="primary" className="mx-2 mb-2 btn-huge" size="lg" onClick={food_retrying} style={{ width: "75%", "font-size": "35px" }}>Fork is empty</Button>
                </Row>
                <Footer />
            </div>
        )
    }

    // State 4: "Waiting for user to open mouth"
    else if (currentStateVal.feeding_status == constants.States[4]) {
        return (
            <div style={{ "display": "block", "width": "100%", "height": "115vh", "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                <h1 className=" text-center txt-huge" style={{ "font-size": "40px" }}>Readiness for Your Bite</h1>
                {/* <h5>{constants.States[4]}</h5> */}

                {isConnected ? <div style={{ "display": "inline" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "inline" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}

                <div style={{ "display": "inline" }}><Button className="cancelBut" style={{ "font-size": "24px" }} onClick={() => changeState(constants.States[1])}>🗑 Cancel Bite</Button></div>

                <p class="transmessage" style={{ "margin-bottom": "0px" }}>Click the below button to indicate that you are ready for your bite.</p>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="primary" className="mx-2 mb-2 btn-huge" size="lg" onClick={ready_for_bite_clicked} style={{ width: "75%", "font-size": "35px" }}>Ready for a Bite</Button>
                </Row>
                <Footer />
            </div>
        )
    }

    // State 5: "Moving closer to your mouth"
    else if (currentStateVal.feeding_status == constants.States[5]) {
        return (
            <div style={{ "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                {isConnected ? <div style={{ "display": "inline-block" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "inline-block" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}

                <div style={{ "display": "inline-block" }}><Button className="cancelBut" style={{ "font-size": "24px", "margin-top": "0px" }} onClick={() => changeState(constants.States[1])}>🗑 Cancel Bite</Button></div>

                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ?
                        <div>
                            <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={moving_closer_mouth_done}>{constants.States[5]}_DONE</Button>
                            {
                                message == "moving_closer_mouth_done"
                                    ? moving_closer_mouth_done()
                                    : <div>
                                        <h1 id={constants.States[5]} className="waitingMsg">Waiting for Robot to move closer to your mouth...</h1>
                                    </div>
                            }
                        </div>
                        : message == "moving_closer_mouth_done"
                            ? moving_closer_mouth_done()
                            : <div>
                                <h1 id={constants.States[5]} className="waitingMsg">Waiting for Robot to move closer to your mouth...</h1>
                            </div>
                    }
                </Row>
                <Footer />
            </div>
        )
    }
    // State 6: "Waiting for user to complete the bite"
    else if (currentStateVal.feeding_status == constants.States[6]) {
        return (
            <div style={{ "display": "block", "width": "100%", "height": "105vh", "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                <h1 className=" text-center txt-huge" style={{ "font-size": "40px" }}>Complete Your Bite</h1>
                {/* <h5>{constants.States[6]}</h5> */}

                {isConnected ? <div style={{ "display": "inline-block" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "inline-block" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}

                <div style={{ "display": "inline-block" }}><Button className="cancelBut" style={{ "font-size": "24px", "margin-top": "0px" }} onClick={() => changeState(constants.States[1])}>🗑 Cancel Bite</Button></div>

                <p class="transmessage" style={{ "margin-bottom": "0px" }}>Click the below button to indicate the completion of your bite.</p>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="primary" className="mx-2 mb-2 btn-huge" size="lg" onClick={done_with_bite_clicked} style={{ width: "75%", "font-size": "35px" }}>Done with Bite</Button>
                </Row>
                <Footer />
            </div>
        )
    }

    // State 9: "Arm is getting stowed"
    else if (currentStateVal.feeding_status == constants.States[9]) {
        return (
            <div style={{ "overflow-x": "hidden", "overflow-y": "auto" }} className="outer">
                {isConnected ? <div style={{ "display": "inline-block" }}><p class="connectedDiv" style={{ "font-size": "24px" }}>🔌 connected</p></div> : <div style={{ "display": "inline-block" }}><p class="notConnectedDiv" style={{ "font-size": "24px" }}>⛔ not connected</p></div>}

                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ?
                        <div>
                            <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={done_with_arm_stowing}>{constants.States[9]}_DONE</Button>
                            {
                                message == "done_with_arm_stowing"
                                    ? done_with_arm_stowing()
                                    : <div>
                                        <h1 id={constants.States[9]} className="waitingMsg">Waiting for Robot to stow the arm away...</h1>
                                        <p class="transmessage" style={{ "margin-bottom": "10px", "margin-top": "0px", "font-size": "24px" }}>That was a delicious 😋 meal! Cheers 🥂 to your good health! </p>
                                    </div>
                            }
                        </div>
                        : message == "done_with_arm_stowing"
                            ? done_with_arm_stowing()
                            : <div>
                                <h1 id={constants.States[9]} className="waitingMsg">Waiting for Robot to stow the arm away...</h1>
                            </div>
                    }
                </Row>
                <Footer />
            </div>
        )
    }
}

export default Home;