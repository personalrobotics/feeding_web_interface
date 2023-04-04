import React, { useState }from "react";
import Button from 'react-bootstrap/Button';
import ToggleButtonGroup from 'react-bootstrap/ToggleButtonGroup';
import ToggleButton from 'react-bootstrap/ToggleButton';
import Row from 'react-bootstrap/Row';
import Form from 'react-bootstrap/Form';
import styled from 'styled-components';
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

const btn = styled.button`
    background-color: purple;
    color: white;
    font-size: 25px;
    padding: 10px 20px;
    border-radius: 5px;
    margin: 10px 0px;
    cursor: pointer;
`;

const ButtonToggle = styled(btn)`
  opacity: 0.6;
  ${({ active }) =>
    active &&
    `
    opacity: 1;
  `}
`;

const ButtonGroup = styled.div`
  display: flex;
`;

const positions = ['Side Feeding', 'Front Feeding'];
const readiness = ['Open Mouth', 'Say "I am Ready"', 'Press Button'];
const selection = ['A picture of plate', 'Scrollable list of food'];
const food = ['Name', 'Location', 'Pixel'];
const interaction = ['Voice', 'Switch'];
const video = ['ON', 'OFF'];
const plate = ['ON', 'OFF'];
let pos = 'Side Feeding';

// COOKIES LOGIC
/*
// settings page will have default button values
// if user changes button values and clicks on save, the new button values will be recorded in cookies
// if cookies are present, the settings will boot up with those; otherwise default values will be loaded
// If you set a new cookie, older cookies are not overwritten. 
// The new cookie is added to document.cookie; so if you want to find the value of one specified cookie, 
// you must write a JavaScript function that searches for the cookie value in the cookie string.

// global values
let pos = 'Side Feeding';
let ready = 'Open Mouth';
let select = 'A picture of plate';
let choose = 'Name';
let interact = 'Voice';
let vid = 'ON';
let locator = 'ON';

// if save clicked, update global values from above and activate buttons accordingly, keep that going until next save click
// cookies won't be lost until a expire date
// not sure if onclick will work in save button as already href exists, thus changing cancel to exit
function cookies_update() {
  pos = 'Side Feeding';
  ready = 'Open Mouth';
  select = 'A picture of plate';
  choose = 'Name';
  interact = 'Voice';
  vid = 'ON';
  locator = 'ON';
}

// if active buttons change, those are updated in let temp buttons through change functions
// only if save button is clicked, cookies true and created => setCookie
// next boot up in settings, load setActive from last cookie do => checkCookie & getCookie 
// in next change in settings, another new cookie created => setCookie with different session name
*/

function ToggleGroupPositions() {
    // The first value active is our current state.
    // The second value setActive is the function that is used to update our state.
    // Lastly, we set the initial state to the first postion: useState(positions[0])
    const [active, setActive] = useState(positions[0]);
    return (
      <ButtonGroup>
        {positions.map(type => (
          <ButtonToggle
            key={type}
            active={active === type}
            onClick={() => changePos(type, setActive)}
          >
            {type}
          </ButtonToggle>
        ))}
      </ButtonGroup>
    );
  }

  function changePos(type, setActive) {
    setActive(type); 
    pos = type;
  }

  function ToggleGroupReadiness() {
    const [active, setActive] = useState(readiness[0]);
    return (
      <ButtonGroup>
        {readiness.map(type => (
          <ButtonToggle
            key={type}
            active={active === type}
            onClick={() => setActive(type)}
          >
            {type}
          </ButtonToggle>
        ))}
      </ButtonGroup>
    );
  }

  function ToggleGroupSelection() {
    const [active, setActive] = useState(selection[0]);
    return (
      <ButtonGroup>
        {selection.map(type => (
          <ButtonToggle
            key={type}
            active={active === type}
            onClick={() => setActive(type)}
          >
            {type}
          </ButtonToggle>
        ))}
      </ButtonGroup>
    );
  }

  function ToggleGroupFood() {
    const [active, setActive] = useState(food[0]);
    return (
      <ButtonGroup>
        {food.map(type => (
          <ButtonToggle
            key={type}
            active={active === type}
            onClick={() => setActive(type)}
          >
            {type}
          </ButtonToggle>
        ))}
      </ButtonGroup>
    );
  }

  function ToggleGroupMedium() {
    const [active, setActive] = useState(interaction[0]);
    return (
      <ButtonGroup>
        {interaction.map(type => (
          <ButtonToggle
            key={type}
            active={active === type}
            onClick={() => setActive(type)}
          >
            {type}
          </ButtonToggle>
        ))}
      </ButtonGroup>
    );
  }

  function ToggleGroupPlate() {
    const [active, setActive] = useState(plate[0]);
    return (
      <ButtonGroup>
        {plate.map(type => (
          <ButtonToggle
            key={type}
            active={active === type}
            onClick={() => setActive(type)}
          >
            {type}
          </ButtonToggle>
        ))}
      </ButtonGroup>
    );
  }

  function ToggleGroupVideo() {
    const [active, setActive] = useState(video[0]);
    return (
      <ButtonGroup>
        {video.map(type => (
          <ButtonToggle
            key={type}
            active={active === type}
            onClick={() => setActive(type)}
          >
            {type}
          </ButtonToggle>
        ))}
      </ButtonGroup>
    );
  }

const Settings = () => {
    return (
        <div>
            <style type="text/css">
                {`
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
                    font-size: 300%;
                }

                `}
            </style>
            <h1 style={{ textAlign: "center", "font-size": "40px" }} className='txt-huge'>⚙ Settings</h1>

            <Row className="justify-content-center mx-1 my-2" >
                <Form.Label style={{ fontSize: "30px" }}>Your preferred feeding position: </Form.Label>
                <ToggleGroupPositions /> 
            </Row>

            <Row className="justify-content-center mx-1 my-2">
                <Form.Label style={{ fontSize: "30px" }}>Your preferred way of indicating readiness for a bite: </Form.Label>
               <ToggleGroupReadiness /> 
            </Row>

            <Row className="justify-content-center mx-1 my-2">
                <Form.Label style={{ fontSize: "30px" }}>Your preferred way for indicating selection of food: </Form.Label>
                <ToggleGroupSelection />
            </Row>

            <Row className="justify-content-center mx-1 my-2">
                <Form.Label style={{ fontSize: "30px" }}>Your preferred way of choosing food: </Form.Label>
                <ToggleGroupFood />
            </Row>

            <Row className="justify-content-center mx-1 my-2">
                <Form.Label style={{ fontSize: "30px" }}>Your preferred interaction medium: </Form.Label>
                <ToggleGroupMedium />
            </Row>

            <Row className="justify-content-center mx-1 my-2">
                <Form.Label style={{ fontSize: "30px" }}>Video from robot's camera: </Form.Label>
                <ToggleGroupVideo/>
            </Row>

            <Row className="justify-content-center mx-1 my-2">
                <Form.Label style={{ fontSize: "30px" }}>Plate locator functionality: </Form.Label>
                <ToggleGroupPlate />
            </Row>
             
            <button type="button" class="btn btn-success" style={{"margin-left": '28%', "fontSize": "30px", "margin-right": '3%', "margin-down": '30px'}}>Save</button>
            <button type="button" class="btn btn-danger" style={{"fontSize": "30px"}}><a href="/" style={{"textDecoration": 'none', "color": 'white'}}>Exit</a></button>
            
        </div>
    );
}

export default Settings;