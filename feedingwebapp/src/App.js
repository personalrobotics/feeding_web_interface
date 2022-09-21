import './App.css';
import React, {useRef} from 'react';
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
import { useROS } from 'react-ros';

import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

import Home from "./Pages/Home/HomeWrapper";
import Settings from "./Pages/Settings/Settings";
import Transition from "./Pages/Transitions/Transition";
import ROSPage from "./Pages/rosSubs";
import Footer from "./Pages/Footer/Footer";

function getWidthHeight(imageWidth = 640, imageHeight = 480) {
  let imageAspectRatio = imageWidth / imageHeight;
  let windowWidth = window.innerWidth;
  let windowHeight = window.innerHeight;
  console.log(windowWidth, windowHeight)
  let returnWidth = 0;
  let returnHeight = 0;
  if (windowWidth / windowHeight > imageAspectRatio) {
    returnHeight = windowHeight - 70;
    returnWidth = imageAspectRatio * returnHeight;
  } else {
    returnWidth = windowWidth; 
    returnHeight = (1 / imageAspectRatio) * returnWidth;
  }
  console.log(returnWidth)
  console.log(returnHeight)
  return {
    "width": returnWidth,
    "height": returnHeight
  }
}

function MyVerticallyCenteredModal(props) {
  const ref = useRef(null);
  return (
    <Modal
      {...props}
      size="lg"
      aria-labelledby="contained-modal-title-vcenter"
      backdrop="static"
      keyboard={false}
      centered
      id="myModal"
      ref={ref}
      fullscreen={true}
    >
      <Modal.Header closeButton>
        <Modal.Title id="contained-modal-title-vcenter">
          Live Video
        </Modal.Title>
      </Modal.Header>
      <Modal.Body style={{"paddingLeft": "10px", "overflow": "hidden"}}>
        <iframe src={`http://localhost:8080/stream?topic=/camera/color/image_raw&default_transport=compressed&width=${Math.round(getWidthHeight().width) - 30}&height=${Math.round(getWidthHeight().height)}&quality=20`}
          frameborder='0'
          allow='autoplay; encrypted-media'
          allowfullscreen
          title='video'
          style={{"width": getWidthHeight().width, "height": getWidthHeight().height}}
        />
      </Modal.Body>
    </Modal>
  );
}

function App() {
  const [modalShow, setModalShow] = React.useState(false);
  const currentStateVal = useStore((state) => state.defaultState);

  const notifyTimeout = () => toast("Please complete or terminate the feeding process to access Settings.");

  return (
    <>

      <Router>
        {console.log(constants.States[7])}
        <Navbar collapseOnSelect expand="lg" bg="dark" variant="dark">

          <Navbar id="responsive-navbar-nav">
            <Nav className="me-auto">
              <Nav.Link className="text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2" href="/" style={{"font-size": "175%"}}>Home</Nav.Link>
              {(currentStateVal.feeding_status == constants.States[7] || currentStateVal.feeding_status == constants.States[8]) ?
                <Nav.Link className="text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2" style={{"font-size": "175%"}} href="/settings">Settings</Nav.Link>
                : <div onClick={notifyTimeout}><Nav.Link disabled className="text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2" style={{"font-size": "175%"}} href="/settings">Settings</Nav.Link></div>
              }
            </Nav>
            <Nav>
              <Nav.Link onClick={() => setModalShow(true)} className="text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2" style={{"font-size": "175%"}}>Video</Nav.Link>
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

        <ToastContainer style={{"font-size": "28px"}}/>
        
      </Router >
    </>
  );
}

export default App;
