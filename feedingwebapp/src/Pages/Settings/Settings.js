import React from "react";

import ToggleButtonGroup from 'react-bootstrap/ToggleButtonGroup';
import ToggleButton from 'react-bootstrap/ToggleButton';
import Row from 'react-bootstrap/Row';
import Form from 'react-bootstrap/Form';
import Footer from "../Footer/Footer";

const Settings = () => {
    return (
        <div>
            <h1 style={{ textAlign: "center" }} className='txt-huge'>Settings</h1>

            <Row className="justify-content-center mx-1 my-2" >
                <Form.Label style={{fontSize: "30px"}}>Your preferred feeding style: </Form.Label>
                <ToggleButtonGroup type="radio" name="options" defaultValue={1}>
                    <ToggleButton className="border border-white btn-huge" style={{paddingLeft: '0px', paddingRight: '0px'}} id="tbg-radio-2" value={2}>
                        Side Feeding
                    </ToggleButton>
                    <ToggleButton className="border border-white btn-huge" style={{paddingLeft: '0px', paddingRight: '0px'}} id="tbg-radio-3" value={3}>
                        Front Feeding
                    </ToggleButton>
                </ToggleButtonGroup>
            </Row>

            <Row className="justify-content-center mx-1 my-2">
                <Form.Label style={{fontSize: "30px"}}>Your preferred method of indicating readiness for a bite: </Form.Label>
                <ToggleButtonGroup type="radio" name="options2" defaultValue={1} >
                    <ToggleButton className="border border-white btn-huge" style={{paddingLeft: '0px', paddingRight: '0px'}} id="tbg-radio-4" value={2}>
                        Open Mouth
                    </ToggleButton>
                    <ToggleButton className="border border-white btn-huge" style={{paddingLeft: '0px', paddingRight: '0px'}} id="tbg-radio-5" value={3}>
                        Say "I'm done"
                    </ToggleButton>
                    <ToggleButton className="border border-white btn-huge" style={{paddingLeft: '0px', paddingRight: '0px'}} id="tbg-radio-6" value={3}>
                        Press button
                    </ToggleButton>
                </ToggleButtonGroup>
            </Row>

            <Row className="justify-content-center mx-1 my-2" style={{ paddingBottom: '35vh'}}>
                <Form.Label style={{fontSize: "30px"}}>Your preferred method for indicating selection of food: </Form.Label>
                <ToggleButtonGroup type="radio" name="options2" defaultValue={1} >
                    <ToggleButton className="border border-white btn-huge" style={{paddingLeft: '0px', paddingRight: '0px'}} id="tbg-radio-4" value={2}>
                        A picture of plate
                    </ToggleButton>
                    <ToggleButton className="border border-white btn-huge" style={{paddingLeft: '0px', paddingRight: '0px'}} id="tbg-radio-5" value={3}>
                        Scrollable list of food
                    </ToggleButton>
                </ToggleButtonGroup>
            </Row>

            <Footer />

        </div>
    );
}

export default Settings;