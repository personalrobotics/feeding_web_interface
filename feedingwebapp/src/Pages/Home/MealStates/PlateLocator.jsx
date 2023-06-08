// React Imports
import React, { useCallback, useEffect, useState } from 'react'
import Button from 'react-bootstrap/Button'
import { useMediaQuery } from 'react-responsive'
import { View } from 'react-native'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { REALSENSE_WIDTH, REALSENSE_HEIGHT } from '../../Constants'
import { useWindowSize, convertRemToPixels, scaleWidthHeightToWindow, showVideo } from '../../../helpers'
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
  // Get robot motion flag for plate locator
  const setTeleopIsMoving = useGlobalState((state) => state.setTeleopIsMoving)
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Factor to modify video size in landscape which has less space than portrait
  let landscapeSizeFactor = 0.9
  // Define margin for video
  const margin = convertRemToPixels(1)
  // Get current window size
  let windowSize = useWindowSize()
  // Define variables for width and height of video
  const [width, setWidth] = useState(windowSize[0])
  const [height, setHeight] = useState(windowSize[1])

  // Update the image size when the screen changes size.
  useEffect(() => {
    // Get the size of the robot's live video stream.
    let { width: widthUpdate, height: heightUpdate } = scaleWidthHeightToWindow(
      windowSize,
      REALSENSE_WIDTH,
      REALSENSE_HEIGHT,
      margin,
      margin,
      margin,
      margin
    )
    setWidth(widthUpdate)
    setHeight(heightUpdate)
  }, [windowSize, margin])

  /**
   * Callback function for when the user presses one of the buttons to teleop
   * the robot.
   *
   * TODO: Implement this when ROS is connected to the robot!
   */
  const cartesianControlCommandReceived = useCallback(
    (event) => {
      let direction = event.target.value
      setTeleopIsMoving(true)
      console.log('cartesianControlCommandReceived', direction)
    },
    [setTeleopIsMoving]
  )

  /**
   * Callback function for when the user indicates that they are done
   * teleoperating the robot.
   */
  const doneClicked = useCallback(() => {
    console.log('doneClicked')
    setTeleopIsMoving(false)
    setMealState(MEAL_STATE.U_BiteSelection)
  }, [setMealState, setTeleopIsMoving])

  /**
   * Get the done button to click when locating plate is done.
   *
   * @returns {JSX.Element} the done button
   */
  const doneButton = useCallback(() => {
    return (
      <center>
        <Button variant='success' onClick={doneClicked} style={{ width: '150px', fontSize: '25px' }}>
          ✅ Done
        </Button>
      </center>
    )
  }, [doneClicked])

  /**
   * An array of directional buttons for the user to teleoperate the robot, and a
   * button for the user to indicate that they are done teleoperating the robot.
   *
   * @returns {JSX.Element} the directional arrow buttons
   */
  const directionalArrows = useCallback(() => {
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
  }, [cartesianControlCommandReceived])

  // Render the component
  return (
    <div style={{ overflowX: 'hidden', overflowY: 'auto' }} className='justify-content-center'>
      {isPortrait ? (
        <React.Fragment>
          <center>{showVideo(props.webVideoServerURL, width, height, null)}</center>
          {directionalArrows()}
          {doneButton()}
        </React.Fragment>
      ) : (
        <View style={{ flexDirection: 'row', justifyContent: 'center', alignItems: 'center', marginTop: '0.5px' }}>
          <View style={{ alignItems: 'center' }}>
            {showVideo(props.webVideoServerURL, width * landscapeSizeFactor, height * landscapeSizeFactor, null)}
          </View>
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
