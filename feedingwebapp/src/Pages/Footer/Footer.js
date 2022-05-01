import React from "react";
import { MDBFooter } from 'mdb-react-ui-kit';
import Button from 'react-bootstrap/Button';
import * as constants from '../Constants';
import useStore from "../useStore";

const Footer = () => {
    const currentStateVal = useStore((state) => state.defaultState);
    const changeState = useStore((state) => state.changeState);

    const changeStatus = () => {
        changeState(constants.States[8]);
        console.log(currentStateVal.feeding_status);
    };

    return (
        <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
            <div className='text-center p-3' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)' }}>
                <Button className="bg-danger" size="lg" onClick={() => changeStatus()}>Emergency Stop</Button>
            </div>
        </MDBFooter>
    );
}

export default Footer;