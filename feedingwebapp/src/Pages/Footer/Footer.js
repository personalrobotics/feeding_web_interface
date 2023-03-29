import { React, useState, useRef } from "react";
import { MDBFooter } from 'mdb-react-ui-kit';
import Button from 'react-bootstrap/Button';
import * as constants from '../Constants';
import useStore from "../useStore";
import Modal from 'react-bootstrap/Modal';


const Footer = () => {
    const currentStateVal = useStore((state) => state.defaultState);
    const changeState = useStore((state) => state.changeState);
    const [pause, setPause] = useState(false);
    const [modalShow, setModalShow] = useState(false);

    // change status function for when Pause is pressed
    const changeStatus = () => {
        setPause(!pause);
        setModalShow(!modalShow);
        // TODO:make sure to send the message to the robot that there has been a pause!!

        console.log(currentStateVal.feeding_status);
    };

    return (
        <>
            <Modal
                size="lg"
                aria-labelledby="contained-modal-title-vcenter"
                backdrop="static"
                keyboard={false}
                centered
                id="myModal"
                fullscreen={true}
                show={modalShow}
                onHide={changeStatus}
            >
                <Modal.Header>
                    <Modal.Title id="contained-modal-title-vcenter">
                        ⏸️ Paused!
                    </Modal.Title>
                </Modal.Header>
                <Modal.Body style={{ "paddingLeft": "10px", "overflow": "hidden" }}>
                <p class="transmessage" style={{ "margin-bottom": "10px", "margin-top": "0px", "font-size": "24px" }}>Go ahead and resume the feeding session when you are ready.</p>
                    <Button className="bg-warning rounded btn-hugeE" style={{ "font-size": "50px", "margin-left": "8%" }} size="lg" onClick={() => changeStatus()}>▶️ Resume</Button>
                </Modal.Body>
            </Modal>

            <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
                <div className='text-center p-3' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)' }}>
                    {<Button className="bg-warning rounded btn-hugeE" style={{ "font-size": "50px" }} size="lg" onClick={() => changeStatus()}>⏸️ Pause</Button>}
                </div>
            </MDBFooter>
        </>
    );
}

export default Footer;