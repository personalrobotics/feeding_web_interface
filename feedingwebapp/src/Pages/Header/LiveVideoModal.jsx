// React imports
import React, { useRef } from 'react'
// The Modal is a screen that appears on top of the main app, and can be toggled
// on and off.
import Modal from 'react-bootstrap/Modal'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'

// Local imports
import { convertRemToPixels } from '../../helpers'
import VideoFeed from '../Home/VideoFeed'

/**
 * The LiveVideoModal displays to the user the live video feed from the robot.
 *
 * TODO: Consider what will happen if the connection to ROS isn't working.
 */
function LiveVideoModal(props) {
  // Variables to render the VideoFeed
  const modalBodyRef = useRef(null)
  const margin = convertRemToPixels(1)

  return (
    <Modal
      show={props.show}
      onHide={props.onHide}
      size='lg'
      aria-labelledby='contained-modal-title-vcenter'
      backdrop='static'
      keyboard={false}
      centered
      id='liveVideoModal'
      fullscreen={true}
    >
      <Modal.Header closeButton>
        <Modal.Title id='contained-modal-title-vcenter' style={{ fontSize: '3vh' }}>
          Live Video
        </Modal.Title>
      </Modal.Header>
      <Modal.Body ref={modalBodyRef} style={{ overflow: 'hidden' }}>
        <center>
          <VideoFeed
            webVideoServerURL={props.webVideoServerURL}
            parent={modalBodyRef}
            marginTop={margin}
            marginBottom={margin}
            marginLeft={margin}
            marginRight={margin}
          />
        </center>
      </Modal.Body>
    </Modal>
  )
}
LiveVideoModal.propTypes = {
  // Whether or not the modal is visible
  show: PropTypes.bool.isRequired,
  // Callback function for when the modal is hidden
  onHide: PropTypes.func.isRequired,
  // The URL of the ROS web video server
  webVideoServerURL: PropTypes.string.isRequired
}

export default LiveVideoModal
