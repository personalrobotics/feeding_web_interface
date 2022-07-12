import React, { useCallback, useEffect } from "react";

import { ROS } from 'react-ros';
import { useROS } from 'react-ros'

import Button from 'react-bootstrap/Button';
import Row from 'react-bootstrap/Row';
import useStore from "../useStore";
import * as constants from '../Constants';
import Modal from 'react-bootstrap/Modal';
import Footer from "../Footer/Footer";
import { Container } from "react-bootstrap";
import { useForm } from "form-control-react";

const debug = true;
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

function Home() {
    let { isConnected, topics, url, changeUrl, toggleConnection } = useROS();
    url = "ws://localhost:9090";
    console.log(topics);

    const [modalShow, setModalShow] = React.useState(false);

    const currentStateVal = useStore((state) => state.defaultState);
    console.log(currentStateVal);
    console.log(topics)
    const changeState = useStore((state) => state.changeState);

    function start_feeding_clicked() {
        url="ws://localhost:9090"
        changeUrl(url)
        toggleConnection();
        console.log(url);
        console.log("start_feeding_clicked");
        // State 1: "Moving above the plate"
        // changeState(constants.States[1]);
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

    if (currentStateVal.feeding_status == constants.States[7] || currentStateVal.feeding_status == constants.States[8]) {
        return (
            <ROS>
                <div>
                    <h1 className="text-center txt-huge">Home</h1>
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
                </div>
            </ROS>
        )
    }
    // State 1: "Moving above the plate"
    else if (currentStateVal.feeding_status == constants.States[1]) {
        return (
            <ROS>
                <h1 className="text-center txt-huge">Moving above the Plate</h1>
                <h5>{constants.States[1]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ? <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={moving_above_plate_done}>{constants.States[1]}_DONE</Button>
                        : <br />
                    }
                </Row>
            </ROS>
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
            </div>
        );
    }
    // State 3: "Acquiring Food Item"
    else if (currentStateVal.feeding_status == constants.States[3]) {
        return (
            <div>
                <h1 className=" text-center txt-huge">Acquiring the selected food</h1>
                <h5>{constants.States[3]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ? <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={acquiring_food_item_done}>{constants.States[3]}_DONE</Button>
                        : <br />
                    }
                </Row>
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
            </div>
        )
    }
    // State 5: "Moving closer to your mouth"
    else if (currentStateVal.feeding_status == constants.States[5]) {
        return (
            <div>
                <h1 className=" text-center txt-huge">Moving closer to your mouth</h1>
                <h5>{constants.States[5]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    {debug
                        ? <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={moving_closer_mouth_done}>{constants.States[5]}_DONE</Button>
                        : <br />
                    }
                </Row>
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
            </div>
        )
    }

}

export default Home;