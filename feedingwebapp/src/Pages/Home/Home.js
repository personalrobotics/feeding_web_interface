import React from "react";
import Button from 'react-bootstrap/Button';
import Row from 'react-bootstrap/Row';
import useStore from "../useStore";
import * as constants from '../Constants';
import Modal from 'react-bootstrap/Modal';
import Footer from "../Footer/Footer";
import { Container } from "react-bootstrap";

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

const Home = () => {
    const [modalShow, setModalShow] = React.useState(false);

    const currentStateVal = useStore((state) => state.defaultState);
    console.log(currentStateVal);
    const changeState = useStore((state) => state.changeState);

    if (currentStateVal.feeding_status == constants.States[7] || currentStateVal.feeding_status == constants.States[8]) {
        return (
            <>
                <div>
                    <h1 className="text-center txt-huge">Home</h1>
                    <Row xs={1} md={1} className="justify-content-center mx-2 my-2">
                        <Button variant="primary" className="btn-huge" size="lg" onClick={function () {
                            changeState(constants.States[1]);
                            console.log(currentStateVal);
                        }}>Start Feeding</Button>
                    </Row>
                    <MyVerticallyCenteredModal
                        show={modalShow}
                        onHide={() => setModalShow(false)}
                    />
                </div>
            </>
        )
    } else if (currentStateVal.feeding_status == constants.States[1]) {
        return (
            <div>
                <h1 className="text-center txt-huge">Moving above the Plate</h1>
                <h5>{constants.States[1]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={function () {
                        changeState(constants.States[2]);
                    }}>{constants.States[1]}_DONE</Button>
                </Row>
            </div>
        )
    } else if (currentStateVal.feeding_status == constants.States[2]) {
        return (
            <div>
                <h1 className="text-center txt-huge">Select a food item</h1>
                <Row xs={1} md={3} lg={4} className="justify-content-center mx-1 my-2" style={{ paddingBottom: '35vh'}}>
                    {food.map((value, i) => (
                        <Button variant="primary" className="mx-2 mb-2 btn-huge" style={{paddingLeft: "0px", paddingRight:"0px"}}size="lg" onClick={function () {
                            changeState(constants.States[3]);
                        }}>{value}</Button>
                    ))}
                </Row>
            </div>
        );
    } else if (currentStateVal.feeding_status == constants.States[3]) {
        return (
            <div>
                <h1 className=" text-center txt-huge">Acquiring the selected food</h1>
                <h5>{constants.States[3]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={function () {
                        changeState(constants.States[4]);
                    }}>{constants.States[3]}_DONE</Button>
                </Row>
            </div>
        )
    } else if (currentStateVal.feeding_status == constants.States[4]) {
        return (
            <div>
                <h1 className=" text-center txt-huge">Click when ready for a bite</h1>
                <h5>{constants.States[4]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="primary" className="mx-2 mb-2 btn-huge" size="lg" onClick={function () {
                        changeState(constants.States[5]);
                    }}>Ready for a Bite</Button>
                </Row>
            </div>
        )
    } else if (currentStateVal.feeding_status == constants.States[5]) {
        return (
            <div>
                <h1 className=" text-center txt-huge">Moving closer to your mouth</h1>
                <h5>{constants.States[5]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="secondary" className="mx-2 mb-2" size="lg" onClick={function () {
                        changeState(constants.States[6]);
                    }}>{constants.States[5]}_DONE</Button>
                </Row>
            </div>
        )
    } else if (currentStateVal.feeding_status == constants.States[6]) {
        return (
            <div>
                <h1 className=" text-center txt-huge">Click when you are done with your bite</h1>
                <h5>{constants.States[6]}</h5>
                <Row className="justify-content-center mx-auto my-2 w-75">
                    <Button variant="primary" className="mx-2 mb-2 btn-huge" size="lg" onClick={function () {
                        changeState(constants.States[1]);
                    }}>Done with Bite</Button>
                </Row>
            </div>
        )
    }

}

export default Home;