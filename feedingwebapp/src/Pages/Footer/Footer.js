import React from "react";
import { MDBFooter } from 'mdb-react-ui-kit';
import Button from 'react-bootstrap/Button';
import * as constants from '../Constants';
import useStore from "../useStore";
import { useROS } from "react-ros";

const Footer = () => {
    const currentStateVal = useStore((state) => state.defaultState);
    const changeState = useStore((state) => state.changeState);

    let {toggleConnection} = useROS();

    const changeStatus = () => {
        changeState(constants.States[8]);
        console.log(currentStateVal.feeding_status);
    };

    return (
        <>
            {/* <style type="text/css">
                {`
                @media screen and (max-width: 1000px) and (min-height: 320px) {
                    .btn-huge {
                        font-size: 135%;                        
                    }
                }
                @media screen and (max-width: 1000px) and (min-height: 550px) {
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
                    font-size: 230%;
                }

                `}
            </style> */}
            <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
                <div className='text-center p-3' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)'}}>
                    <Button className="bg-danger rounded btn-hugeE" style={{"font-size": "50px"}} size="lg" onClick={() => changeStatus()}>Emergency Stop</Button>
                </div>
            </MDBFooter>
        </>
    );
}

export default Footer;