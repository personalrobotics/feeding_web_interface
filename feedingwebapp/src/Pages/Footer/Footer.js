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
        <>
            <style type="text/css">
                {`
                @media screen and (max-width: 1000px) {
                    .btn-huge {
                        padding: 15% 20%;
                        font-size: 200%;                        
                    }
                }
                @media screen and (min-width: 1000px) {
                    .btn-huge {
                        padding: 5% 15%;
                        font-size: 150%;                       
                    }
                }
                    .txt-huge {
                        font-size: 300%;
                    }

                `}
            </style>
            <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
                <div className='text-center p-3' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)'}}>
                    <Button className="bg-danger rounded" size="huge" onClick={() => changeStatus()}>Emergency Stop</Button>
                </div>
            </MDBFooter>
        </>
    );
}

export default Footer;