import React from "react";
import { MDBFooter } from 'mdb-react-ui-kit';
import Button from 'react-bootstrap/Button';

const Footer = () => {
    return (
        <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
            <div className='text-center p-3' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)' }}>
                <Button className="bg-danger">Emergency Stop</Button>
            </div>
        </MDBFooter>
    );
}

export default Footer;