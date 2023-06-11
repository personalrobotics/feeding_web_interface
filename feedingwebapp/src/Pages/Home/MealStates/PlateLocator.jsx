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
  // Indicator of how to arrange screen elements based on orientation
  let dimension = isPortrait ? 'column' : 'row'
  // Define margin for video
  const margin = convertRemToPixels(1)
  // Get current window size
  let windowSize = useWindowSize()
  // text font size
  let textFontSize = isPortrait ? '3vh' : '3vw'
  // done button width
  let doneButtonWidth = isPortrait ? '15vh' : '15vw'
  // button height
  let buttonHeight = isPortrait ? '6vh' : '6vw'
  // arrow button width
  let arrowButtonWidth = isPortrait ? '6vh' : '6vw'
  // Factor to modify video size in landscape which has less space than portrait
  let landscapeSizeFactor = 0.5
  // Define variables for width and height of video
  const [imgWidth, setWidth] = useState(windowSize.width)
  const [imgHeight, setHeight] = useState(windowSize.height)

  // Update the image size when the screen changes size.
  useEffect(() => {
    // Get the size of the robot's live video stream.
    let { width: imgWidthUpdate, height: imgHeightUpdate } = scaleWidthHeightToWindow(
      windowSize,
      REALSENSE_WIDTH,
      REALSENSE_HEIGHT,
      margin,
      margin,
      margin,
      margin
    )
    setWidth(imgWidthUpdate)
    setHeight(imgHeightUpdate)
  }, [windowSize, margin])

  let finalImgWidth = isPortrait ? imgWidth : landscapeSizeFactor * imgWidth
  let finalImgHeight = isPortrait ? imgHeight : landscapeSizeFactor * imgHeight

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
        <Button variant='success' onClick={doneClicked} style={{ width: doneButtonWidth, height: buttonHeight, fontSize: textFontSize }}>
          ✅ Done
        </Button>
      </center>
    )
  }, [doneClicked, textFontSize, buttonHeight, doneButtonWidth])

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
              style={{
                fontSize: textFontSize,
                alignItems: 'right',
                justifyContent: 'center',
                width: arrowButtonWidth,
                height: buttonHeight,
                marginTop: margin
              }}
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
                  style={{
                    fontSize: textFontSize,
                    alignItems: 'center',
                    justifyContent: 'center',
                    width: arrowButtonWidth,
                    height: buttonHeight
                  }}
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
                  style={{
                    fontSize: textFontSize,
                    alignItems: 'left',
                    justifyContent: 'center',
                    width: arrowButtonWidth,
                    height: buttonHeight
                  }}
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
              style={{
                fontSize: textFontSize,
                alignItems: 'center',
                justifyContent: 'center',
                width: arrowButtonWidth,
                marginBottom: margin
              }}
              value='back'
              variant='primary'
            >
              ⬇
            </Button>
          </Row>
        </Container>
      </React.Fragment>
    )
  }, [cartesianControlCommandReceived, arrowButtonWidth, buttonHeight, textFontSize, margin])

  // Render the component
  return (
    <>
      <React.Fragment>
        <View style={{ flex: 1, flexDirection: dimension, justifyContent: 'center', alignItems: 'center', margin: margin }}>
          <View style={{ flex: 5, alignItems: 'center', justifyContent: 'center' }}>
            {showVideo(props.webVideoServerURL, finalImgWidth, finalImgHeight, null)}
          </View>
          <View style={{ flex: 5, alignItems: 'center', justifyContent: 'center' }}>
            {directionalArrows()}
            {doneButton()}
          </View>
        </View>
      </React.Fragment>
    </>
  )
}
PlateLocator.propTypes = {
  // The URL of the web video server
  webVideoServerURL: PropTypes.string.isRequired
}

export default PlateLocator
