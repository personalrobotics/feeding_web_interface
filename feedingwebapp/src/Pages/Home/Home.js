import React, { useCallback, useState, useEffect } from "react";
import './Home.css'
import { ROS } from 'react-ros';
import { useROS } from 'react-ros'
import ScriptTag from 'react-script-tag';

import { RosConnection } from './roslib.min.js';

import Button from 'react-bootstrap/Button';
import Row from 'react-bootstrap/Row';
import useStore from "../useStore";
import * as constants from '../Constants';
import Modal from 'react-bootstrap/Modal';
import Footer from "../Footer/Footer";
import { Container } from "react-bootstrap";
import { useForm } from "form-control-react";
import ROSLIB from "roslib";


const debug = false;
const food = ["Apple", "Banana", "Carrot", "Cucumber", "Lettuce", "Mango", "Orange", "Pumpkin"];

function MyVerticallyCenteredModal(props) {
    return (
        <>
            <Modal
                {...props}
                size="lg"
                aria-labelledby="contained-modal-title-vcenter"
                backdrop="static"
                keyboard={false}
                centered
            >
                <Modal.Header closeButton>
                    <Modal.Title id="contained-modal-title-vcenter">
                        Live Video
                    </Modal.Title>
                </Modal.Header>
                <Modal.Body>
                    <iframe src='https://www.youtube.com/embed/E7wJTI-1dvQ'
                        frameborder='0'
                        allow='autoplay; encrypted-media'
                        allowfullscreen
                        title='video'
                    />
                </Modal.Body>
            </Modal>
        </>
    );
}

var listener = null;
function Home() {

    let { ros, isConnected, topics, url, changeUrl, toggleConnection, createListener } = useROS();
    console.log(topics)
    var new_topic_from_Robot = {
        path: "/fromRobot",
        msgType: "std_msgs/String",
        type: "topic"
    }

    topics[4] = new_topic_from_Robot

    var new_topic_from_Web = {
        path: "/fromWeb",
        msgType: "std_msgs/String",
        type: "topic"
    }

    topics[5] = new_topic_from_Web

    const [topic, setTopic] = useState('/');
    const [queue, setQueue] = useState(0);
    const [message, setMessage] = useState("Hello")
    const [compression, setCompression] = useState('none');

    // componentDidMount() {
    //     console.log("entered add lib function")
    //     const script1 = document.createElement('script');
    //     script1.src = "http://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js";
    //     script1.async = true;
    //     document.getElementsByTagName('head')[0].appendChild(script1);

    //     const script2 = document.createElement('script');
    //     script2.src = "http://static.robotwebtools.org/roslibjs/current/roslib.min.js"
    //     script2.async = true;
    //     document.getElementsByTagName('head')[0].appendChild(script2);

    //     const script3 = document.createElement("script");
    //     script3.type = "text/javascript"
    //     script3.async = true;
    //     script3.src = "./ScriptsROS.js"
    //     document.getElementsByTagName('head')[0].appendChild(script3);

    //     msg()
    // }

    useEffect(() => {
        handleTopic(topic);
        // var ros = new ROSLIB.Ros({
        //     url : 'ws://localhost:9090'
        //   });
        //   console.log(ros)
    });
    const unsubscribe = () => {
        if (listener) {
            console.log("Unsubscribing");
            listener.unsubscribe();
        }
    }
    const handleTopic = (topicInput) => {
        if (topic !== topicInput) {
            setTopic(topicInput);
            unsubscribe();
            return;
        }
        unsubscribe();
        listener = null;
        for (var i in topics) {
            if (topics[i].path == topicInput) {
                listener = createListener(topics[i].path,
                    topics[i].msgType,
                    Number(queue),
                    compression);
                break;
            }
        }
        if (listener) {
            console.log("Subscribing to messages...");
            listener.subscribe(handleMsg);
            console.log(handleMsg)
        } else {
            console.log("Topic '" + topic + "' not found...make sure to input the full topic path - including the leading '/'");
        }
    }
    const handleQueue = (queueInput) => {
        setQueue(queueInput);
    }
    const handleCompression = (compInput) => {
        setCompression(compInput);
    }
    const handleMsg = (msg) => {
        console.log(msg);
        setMessage(msg["data"]);
        console.log(message)
    }
    const [modalShow, setModalShow] = React.useState(false);

    const currentStateVal = useStore((state) => state.defaultState);
    console.log(currentStateVal);
    const changeState = useStore((state) => state.changeState);

    function AddLibrary() {

    }

    function start_feeding_clicked() {
        console.log("start_feeding_clicked");
        // State 1: "Moving above the plate"
        changeState(constants.States[1]);
        console.log(isConnected);
    }

    // Temp Button #1
    function moving_above_plate_done() {
        console.log("moving_above_plate_done");
        // State 2: "Feeding"
        changeState(constants.States[2]);
    }

    function food_item_clicked() {
        console.log("food_item_clicked");
        // State 3: "Acquiring Food Item"
        changeState(constants.States[3]);
    }

    // Temp Button #2
    function acquiring_food_item_done() {
        console.log("acquiring_food_item_done");
        // State 4: "Waiting for user to open mouth"
        changeState(constants.States[4]);
    }

    function ready_for_bite_clicked() {
        console.log("ready_for_bite_clicked");
        // State 5: "Moving closer to your mouth"
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
        changeState(constants.States[1]);
    }

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

        var cmdVel = new ROSLIB.Topic({
            ros : ros,
            name : '/cmd_vel',
            messageType : 'geometry_msgs/Twist'
          });

        var twist = new ROSLIB.Message({
            linear : {
              x : 0.1,
              y : 0.2,
              z : 0.3
            },
            angular : {
              x : -0.1,
              y : -0.2,
              z : -0.3
            }
          });
        cmdVel.publish(twist);
    }

    if (currentStateVal.feeding_status == constants.States[7] || currentStateVal.feeding_status == constants.States[8]) {
        return (
            <>
                <div>
                    <ScriptTag type="text/javascript" src="http://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js" />
                    <ScriptTag type="text/javascript" src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js" />
                    {RosConnect()}

                    <h1 className="text-center txt-huge">Home</h1>


                    <Button onClick={toggleConnection}>Connect to ADA</Button>
                    <Button onClick={AddLibrary}>JS Lib</Button>
                    <Row xs={1} md={1} className="justify-content-center mx-2 my-2">
                        <Button variant="primary" size="lg" className="btn-huge" id="#startBtn"
                            onClick={start_feeding_clicked} style={{ width: "75%" }}>Start Feeding</Button>
                    </Row>
                    <b>ROS url input:  </b><input name="urlInput" defaultValue={url} onChange={event => changeUrl(event.target.value)} />  <br />
                    <MyVerticallyCenteredModal
                        show={modalShow}
                        onHide={() => setModalShow(false)}
                    />
                    <button onClick={toggleConnection}>Toggle connect</button>
                    {isConnected ? "connected" : "not connected"}
                    <b>Topic to echo:  </b><input name="topicInput" defaultValue={topic} onChange={event => handleTopic(event.target.value)} />  <br />
                    <Footer />
                </div>
            </>
        )
    }
    // State 1: "Moving above the plate"
    else if (currentStateVal.feeding_status == constants.States[1]) {
        return (
            <div>
                {isConnected ? <div><p class="connectedDiv">connected</p></div> : <div><p class="notConnectedDiv">not connected</p></div>}
                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ? <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={moving_above_plate_done}>{constants.States[1]}_DONE</Button>
                        : message == "moving_above_plate_done"
                            ? moving_above_plate_done()
                            : <div>
                                <h2 id={constants.States[1]}>Waiting for Robot to move above the plate...</h2>
                                <p class="transmessage">Please wait. The robot is currently moving above your plate of food.</p>
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
            <div>
                <h1 className="text-center txt-huge">Select a food item</h1>
                <Row xs={1} md={3} lg={4} className="justify-content-center mx-1 my-2" style={{ paddingBottom: '35vh' }}>
                    {food.map((value, i) => (
                        <Button key={i} variant="primary" className="mx-2 mb-2 btn-huge" style={{ paddingLeft: "0px", paddingRight: "0px" }} size="lg" onClick={food_item_clicked}>{value}</Button>
                    ))}
                </Row>
                <Footer />
            </div>
        );
    }
    // State 3: "Acquiring Food Item"
    else if (currentStateVal.feeding_status == constants.States[3]) {
        return (
            <div>
                {isConnected ? <div><p class="connectedDiv">connected</p></div> : <div><p class="notConnectedDiv">not connected</p></div>}
                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ? <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={acquiring_food_item_done}>{constants.States[3]}_DONE</Button>
                        : message == "acquiring_food_item_done"
                            ? acquiring_food_item_done()
                            : <div>
                                <h2 id={constants.States[3]}>Waiting for Robot to acquire the food item selected...</h2>
                                <p class="transmessage">Please wait. The robot is currently moving to acquire the food you have selected.</p>
                            </div>
                    }
                </Row>
                <Footer />
            </div>
        )
    }
    // State 4: "Waiting for user to open mouth"
    else if (currentStateVal.feeding_status == constants.States[4]) {
        return (
            <div>
                <h1 className=" text-center txt-huge">Click when ready for a bite</h1>
                <h5>{constants.States[4]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="primary" className="mx-2 mb-2 btn-huge" size="lg" onClick={ready_for_bite_clicked}>Ready for a Bite</Button>
                </Row>
                <Footer />
            </div>
        )
    }
    // State 5: "Moving closer to your mouth"
    else if (currentStateVal.feeding_status == constants.States[5]) {
        return (
            <div>
                {isConnected ? <div><p class="connectedDiv">connected</p></div> : <div><p class="notConnectedDiv">not connected</p></div>}
                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ? <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={moving_closer_mouth_done}>{constants.States[5]}_DONE</Button>
                        : message == "moving_closer_mouth_done"
                            ? moving_closer_mouth_done()
                            : <div>
                                <h2 id={constants.States[5]}>Waiting for Robot to move closer to your mouth...</h2>
                                <p class="transmessage">Please wait. The robot is currently moving closer to your mouth.</p>
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
            <div>
                <h1 className=" text-center txt-huge">Click when you are done with your bite</h1>
                <h5>{constants.States[6]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="primary" className="mx-2 mb-2 btn-huge" size="lg" onClick={done_with_bite_clicked}>Done with Bite</Button>
                </Row>
                <Footer />
            </div>
        )
    }

}

export default Home;