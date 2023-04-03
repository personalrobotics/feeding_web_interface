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

function ToggleGroupPositions() {
    const [active, setActive] = useState(positions[0]);
    return (
      <ButtonGroup>
        {positions.map(type => (
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
             
            <button type="button" class="btn btn-success" style={{"margin-left": '28%', "fontSize": "30px", "margin-right": '3%', "margin-down": '30px'}}><a href="/" style={{"textDecoration": 'none', "color": 'white'}}>Save</a></button>
            <button type="button" class="btn btn-danger" style={{"fontSize": "30px"}}><a href="/" style={{"textDecoration": 'none', "color": 'white'}}>Cancel</a></button>
            
        </div>
    );
}

export default Settings;