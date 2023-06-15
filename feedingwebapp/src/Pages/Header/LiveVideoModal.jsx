// React imports
import React, { useEffect, useRef, useState } from 'react'
import { useMediaQuery } from 'react-responsive'
// The Modal is a screen that appears on top of the main app, and can be toggled
// on and off.
import Modal from 'react-bootstrap/Modal'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'

// Local imports
import { REALSENSE_WIDTH, REALSENSE_HEIGHT } from '../Constants'
import { useWindowSize, convertRemToPixels, scaleWidthHeightToWindow, showVideo } from '../../helpers'

/**
 * The LiveVideoModal displays to the user the live video feed from the robot.
 *
 * TODO: Consider what will happen if the connection to ROS isn't working.
 */
function LiveVideoModal(props) {
  const ref = useRef(null)
  // Use the default CSS properties of Modals to determine the margin around
  // the image. This is necessary so the image is scaled to fit the window.
  //
  // NOTE: This must change if the CSS properties of the Modal change.
  //
  // marginTop: bs-modal-header-padding, h4 font size & line height, bs-modal-header-padding, bs-modal-padding
  const marginTop = convertRemToPixels(1 + 1.5 * 1.5 + 1 + 1)
  const marginBottom = convertRemToPixels(1)
  const marginLeft = convertRemToPixels(1)
  const marginRight = convertRemToPixels(1)

  // Get current window size
  let windowSize = useWindowSize()
  // Define variables for width and height of image
  const [imgWidth, setImgWidth] = useState(windowSize.width)
  const [imgHeight, setImgHeight] = useState(windowSize.height)
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })

  /** Factors to modify video size in landscape to fit space
   *
   * TODO: Adjust it accordingly when flexbox with directional arrows implemented
   */
  let landscapeSizeFactor = 0.9

  // Get final image size according to screen orientation
  let finalImgWidth = isPortrait ? imgWidth : landscapeSizeFactor * imgWidth
  let finalImgHeight = isPortrait ? imgHeight : landscapeSizeFactor * imgHeight

  // Update the image size when the screen changes size.
  useEffect(() => {
    // 640 x 480 is the standard dimension of images outputed by the RealSense
    let { width: widthUpdate, height: heightUpdate } = scaleWidthHeightToWindow(
      windowSize,
      REALSENSE_WIDTH,
      REALSENSE_HEIGHT,
      marginTop,
      marginBottom,
      marginLeft,
      marginRight
    )
    setImgWidth(widthUpdate)
    setImgHeight(heightUpdate)
  }, [windowSize, marginTop, marginBottom, marginLeft, marginRight])

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
      ref={ref}
      fullscreen={true}
    >
      <Modal.Header closeButton>
        <Modal.Title id='contained-modal-title-vcenter' style={{ fontSize: isPortrait ? '3vh' : '3vw' }}>
          Live Video
        </Modal.Title>
      </Modal.Header>
      <Modal.Body style={{ overflow: 'hidden' }}>
        <center>{showVideo(props.webVideoServerURL, finalImgWidth, finalImgHeight, null)}</center>
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