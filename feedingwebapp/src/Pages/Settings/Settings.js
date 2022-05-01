import React from "react";

import ToggleButtonGroup from 'react-bootstrap/ToggleButtonGroup';
import ToggleButton from 'react-bootstrap/ToggleButton';
import Row from 'react-bootstrap/Row';
import Form from 'react-bootstrap/Form';
import Footer from "../Footer/Footer";

const Settings = () => {
    return (
        <div>
            <h1 style={{ textAlign: "center" }}>Settings</h1>

            <Row className="justify-content-center mx-1 my-2" >
                <Form.Label style={{fontSize: "25px"}}>Your preferred feeding style: </Form.Label>
                <ToggleButtonGroup type="radio" name="options" defaultValue={1}>
                    <ToggleButton className="border border-white btn-lg" id="tbg-radio-2" value={2}>
                        Side Feeding
                    </ToggleButton>
                    <ToggleButton className="border border-white btn-lg" id="tbg-radio-3" value={3}>
                        Front Feeding
                    </ToggleButton>
                </ToggleButtonGroup>
            </Row>

            <Row className="justify-content-center mx-1 my-2" >
                <Form.Label style={{fontSize: "25px"}}>Your preferred method of indicating readiness for a bite: </Form.Label>
                <ToggleButtonGroup type="radio" name="options2" defaultValue={1}>
                    <ToggleButton className="border border-white btn-lg" id="tbg-radio-4" value={2}>
                        Open Mouth
                    </ToggleButton>
                    <ToggleButton className="border border-white btn-lg" id="tbg-radio-5" value={3}>
                        Say "I'm done"
                    </ToggleButton>
                    <ToggleButton className="border border-white btn-lg" id="tbg-radio-6" value={3}>
                        Press button
                    </ToggleButton>
                </ToggleButtonGroup>
            </Row>

            <Footer />

        </div>
    );
}

export default Settings;