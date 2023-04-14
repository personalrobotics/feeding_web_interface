// React imports
import React, { useRef } from 'react'
// The Modal is a screen that appears on top of the main app, and can be toggled
// on and off.
import Modal from 'react-bootstrap/Modal'

// Local imports
import { REALSENSE_WIDTH, REALSENSE_HEIGHT } from '../Constants'
import { scaleWidthHeightToWindow } from '../../helpers'

/**
 * The LiveVideoModal displays to the user the live video feed from the robot.
 *
 * TODO (amaln): Remove hardcoded values (e.g., 30), consider what will happen if the
 * connection to ROS isn't working, etc.
 */
function LiveVideoModal(props) {
  const ref = useRef(null)
  // 640 x 480 is the standard dimension of images outputed by the RealSense
  let { width: width, height: height } = scaleWidthHeightToWindow(REALSENSE_WIDTH, REALSENSE_HEIGHT)
  return (
    <Modal
      {...props}
      size='lg'
      aria-labelledby='contained-modal-title-vcenter'
      backdrop='static'
      keyboard={false}
      centered
      id='liveVideoModal'
      ref={ref}
      fullscreen={true}
    >
      <Modal.Header closeButton>
        <Modal.Title id='contained-modal-title-vcenter'>Live Video</Modal.Title>
      </Modal.Header>
      <Modal.Body style={{ paddingLeft: '10px', overflow: 'hidden' }}>
        <iframe
          src={`http://localhost:8080/stream?topic=/camera/color/image_raw&default_transport=compressed&width=${
            Math.round(width) - 30
          }&height=${Math.round(height)}&quality=20`}
          frameBorder='0'
          allow='autoplay; encrypted-media'
          allowfullscreen
          title='video'
          style={{ width: width, height: height }}
        />
      </Modal.Body>
    </Modal>
  )
}

export default LiveVideoModal
