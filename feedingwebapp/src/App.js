import './App.css';
import React from 'react';
import {
  BrowserRouter as Router,
  Routes,
  NavLink,
  Route
} from "react-router-dom";

import './App.css';
import 'bootstrap/dist/css/bootstrap.min.css';
import Navbar from 'react-bootstrap/Navbar';
import Container from 'react-bootstrap/Container';
import Nav from 'react-bootstrap/Nav';
import Modal from 'react-bootstrap/Modal';
import useStore from ".//Pages/useStore";
import * as constants from './Pages/Constants';
import Button from 'react-bootstrap/Button';
import { ROS } from 'react-ros';

// import MyVerticallyCenteredModal from 'react-bootstrap/'
// import Modal from 'react-bootstrap/Modal'

import Home from "./Pages/Home/HomeWrapper";
import Settings from "./Pages/Settings/Settings";
import Transition from "./Pages/Transitions/Transition";
import ROSPage from "./Pages/rosSubs";
import Footer from "./Pages/Footer/Footer";

function MyVerticallyCenteredModal(props) {
  return (
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
  );
}

function App() {
  const [modalShow, setModalShow] = React.useState(false);
  const currentStateVal = useStore((state) => state.defaultState);

  return (
    <>

      <Router>
        {console.log(constants.States[7])}
        <Navbar collapseOnSelect expand="lg" bg="dark" variant="dark">

          <Navbar id="responsive-navbar-nav">
            <Nav className="me-auto">
              <Nav.Link className="text-white border border-info rounded mx-1 btn-lg btn-huge p-2" href="/">Home</Nav.Link>
              {(currentStateVal.feeding_status == constants.States[7] || currentStateVal.feeding_status == constants.States[8]) ?
                <Nav.Link className="text-white border border-info rounded mx-1 btn-lg btn-huge p-2" href="/settings">Settings</Nav.Link>
                : <div onClick={() => alert("Settings is disabled when feeding is in progress. Please end the feeding and then you may access the Settings page for any changes.")}><Nav.Link disabled className="text-white border border-info rounded mx-1 btn-lg btn-huge p-2" href="/settings">Settings</Nav.Link></div>
              }

              {/* <Nav.Link className="text-white border border-info rounded mx-1 btn-lg btn-huge p-2" href="/ros">ROS</Nav.Link> */}
            </Nav>
            <Nav>
              <Nav.Link onClick={() => setModalShow(true)} className="text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2">Video</Nav.Link>
            </Nav>
            <MyVerticallyCenteredModal
              show={modalShow}
              onHide={() => setModalShow(false)}
            />
          </Navbar>

        </Navbar>

        <Routes>
          <Route exact path="/" element={<Home />} />
          <Route exact path="/settings" element={<Settings />} />
          <Route exact path="/transition" element={<Transition />} />
          <Route exact path="/ros" element={<ROSPage />} />
        </Routes>
        
      </Router >
    </>
  );
}

export default App;
