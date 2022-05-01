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
import Button from 'react-bootstrap/Button';

// import MyVerticallyCenteredModal from 'react-bootstrap/'
// import Modal from 'react-bootstrap/Modal'

import Home from "./Pages/Home/Home";
import Settings from "./Pages/Settings/Settings";
import Transition from "./Pages/Transitions/Transition";

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

  return (
    <Router>
      <Navbar collapseOnSelect expand="lg" bg="dark" variant="dark">
        <Container>
          <Navbar id="responsive-navbar-nav">
            <Nav className="me-auto">
              <Nav.Link className="text-white border border-info rounded mx-1 btn-lg" href="/">Home</Nav.Link>
              <Nav.Link className="text-white border border-info rounded mx-1 btn-lg" href="/settings">Settings</Nav.Link>
            </Nav>
            <Nav>
              <Nav.Link onClick={() => setModalShow(true)} className="text-dark bg-info border border-info rounded btn-lg">Video</Nav.Link>
            </Nav>
            <MyVerticallyCenteredModal
              show={modalShow}
              onHide={() => setModalShow(false)}
            />
          </Navbar>
        </Container>
      </Navbar>

      <Routes>
        <Route exact path="/" element={<Home />} />
        <Route exact path="/settings" element={<Settings />} />
        <Route exact path="/transition" element={<Transition />} />
      </Routes>
    </Router >
  );
}

export default App;
