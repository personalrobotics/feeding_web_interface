// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'
import { useMediaQuery } from 'react-responsive'
import { View } from 'react-native'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { REALSENSE_WIDTH, REALSENSE_HEIGHT, CAMERA_FEED_TOPIC } from '../../Constants'
import { useWindowSize, convertRemToPixels, scaleWidthHeightToWindow } from '../../../helpers'
import { Col, Row, Container } from 'react-bootstrap'

/**
 * The PlateLocator component appears if the user decides to adjust the position
 * of the fork above the plate before selecting a bite. This component enables
 * the user to teleoperate the robot with Cartesian Control until the plate
 * is satisfactorily in view.
 */
const PlateLocator = (props) => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })

  /**
   * Callback function for when the user presses one of the buttons to teleop
   * the robot.
   *
   * TODO: Implement this when ROS is connected to the robot!
   */
  function cartesianControlCommandReceived(event) {
    let direction = event.target.value
    console.log('cartesianControlCommandReceived', direction)
  }

  /**
   * Callback function for when the user indicates that they are done
   * teleoperating the robot.
   */
  function doneClicked() {
    console.log('doneClicked')
    setMealState(MEAL_STATE.U_BiteSelection)
  }

  // Get the size of the robot's live video stream.
  let landscapeSizeFactor = 0.9
  let size = useWindowSize()
  const margin = convertRemToPixels(1)
  let { width, height } = scaleWidthHeightToWindow(size, REALSENSE_WIDTH, REALSENSE_HEIGHT, margin, margin, margin, margin)

  /**
   * Get the done button to click when locating plate is done.
   *
   * @returns {JSX.Element} the done button
   */
  let doneButton = function () {
    return (
      <center>
        <Button variant='success' onClick={doneClicked} style={{ width: '150px', fontSize: '25px' }}>
          ✅ Done
        </Button>
      </center>
    )
  }

  /**
   * Get the robot's live video stream.
   *
   * @param {number} currentWidth the adjusted width in pixels
   * @param {number} currentHeight the adjusted height in pixels
   *
   * @returns {JSX.Element} the robot's live video stream
   */
  let showVideo = function (currentWidth, currentHeight) {
    return (
      <img
        src={`${props.webVideoServerURL}/stream?topic=${CAMERA_FEED_TOPIC}&width=${Math.round(currentWidth)}&height=${Math.round(
          currentHeight
        )}&quality=20`}
        alt='Live video feed from the robot'
        style={{
          width: currentWidth,
          height: currentHeight,
          display: 'block',
          alignItems: 'center',
          justifyContent: 'center',
          marginTop: '9px'
        }}
      />
    )
  }

  /**
   * An array of directional buttons for the user to teleoperate the robot, and a
   * button for the user to indicate that they are done teleoperating the robot.
   *
   * @returns {JSX.Element} the directional arrow buttons
   */
  let directionalArrows = function () {
    return (
      <React.Fragment>
        <Container fluid>
          <Row className='justify-content-center' noGutters={true}>
            <Button
              onClick={cartesianControlCommandReceived}
              style={{ fontSize: '25px', alignItems: 'right', width: '55px' }}
              value='forward'
              variant='primary'
            >
              ⬆
            </Button>
          </Row>
          <Row className='justify-content-center' style={{ '--bs-gutter-x': '0rem' }} xs='auto'>
            <Col>
              <center>
                <Button
                  onClick={cartesianControlCommandReceived}
                  style={{ fontSize: '25px', alignItems: 'center', width: '55px' }}
                  value='left'
                  variant='primary'
                >
                  ⬅
                </Button>
              </center>
            </Col>
            <Col style={{ width: '55px' }}></Col>
            <Col>
              <center>
                <Button
                  onClick={cartesianControlCommandReceived}
                  style={{ fontSize: '25px', alignItems: 'left', width: '55px' }}
                  value='right'
                  variant='primary'
                >
                  ➡
                </Button>
              </center>
            </Col>
          </Row>
          <Row className='justify-content-center' noGutters={true}>
            <Button
              onClick={cartesianControlCommandReceived}
              style={{ fontSize: '25px', alignItems: 'center', width: '55px', marginBottom: '10px' }}
              value='back'
              variant='primary'
            >
              ⬇
            </Button>
          </Row>
        </Container>
      </React.Fragment>
    )
  }

  // Render the component
  return (
    <div style={{ overflowX: 'hidden', overflowY: 'auto' }} className='justify-content-center'>
      {isPortrait ? (
        <React.Fragment>
          <center>{showVideo(width, height)}</center>{directionalArrows()}{doneButton()}
        </React.Fragment>
      ) : (
        <View style={{ flexDirection: 'row', justifyContent: 'center', alignItems: 'center', marginTop: '0.5px' }}>
          <View style={{ alignItems: 'center' }}>{showVideo(width * landscapeSizeFactor, height * landscapeSizeFactor)}</View>
          <View style={{ alignItems: 'center' }}>
            {directionalArrows()}
            {doneButton()}
          </View>
        </View>
      )}
    </div>
  )
}
PlateLocator.propTypes = {
  // The URL of the web video server
  webVideoServerURL: PropTypes.string.isRequired
}

export default PlateLocator
