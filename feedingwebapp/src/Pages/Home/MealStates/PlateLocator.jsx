// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { REALSENSE_WIDTH, REALSENSE_HEIGHT, CAMERA_FEED_TOPIC } from '../../Constants'
import { convertRemToPixels, scaleWidthHeightToWindow } from '../../../helpers'

/**
 * The PlateLocator component appears if the user decides to adjust the position
 * of the fork above the plate before selecting a bite. This component enables
 * the user to teleoperate the robot with Cartesian Control until the plate
 * is satisfactorily in view.
 */
const PlateLocator = (props) => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)

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
  const margin = convertRemToPixels(1)
  let { width, height } = scaleWidthHeightToWindow(REALSENSE_WIDTH, REALSENSE_HEIGHT, margin, margin, margin, margin)

  // Render the component
  return (
    <div style={{ overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
      {/**
       * Display the live stream from the robot's camera.
       */}
      <center>
        <img
          src={`${props.webVideoServerURL}/stream?topic=${CAMERA_FEED_TOPIC}&width=${Math.round(width)}&height=${Math.round(
            height
          )}&quality=20`}
          alt='Live video feed from the robot'
          style={{ width: width, height: height, display: 'block' }}
        />
      </center>

      {/**
       * An array of buttons for the user to teleoperate the robot, and a
       * button for the user to indicate that they are done teleoperating the
       * robot.
       *
       * TODO: The values for margins should not be hardcoded. Bootstrap's
       * grid should be able to get alignment without fine-tuning of margins.
       */}
      <div className='container'>
        <div className='row'>
          <div className='col'>
            <Button
              onClick={cartesianControlCommandReceived}
              style={{ fontSize: '25px', marginLeft: '15%' }}
              value='forward'
              variant='primary'
            >
              <img
                style={{ width: '44px', height: 'auto' }}
                src='/other_emoji_imgs/upArrow.svg'
                alt='Up_Arrow_img'
                className='center'
              />
            </Button>
          </div>

          <div className='w-100'></div>

          <div className='col-3'>
            <Button onClick={cartesianControlCommandReceived} style={{ fontSize: '25px', marginLeft: '2%' }} value='left' variant='primary'>
              <img
                style={{ width: '44px', height: 'auto' }}
                src='/other_emoji_imgs/leftArrow.svg'
                alt='Left_Arrow_img'
                className='center'
              />
            </Button>
          </div>

          <div className='col'>
            <Button
              onClick={cartesianControlCommandReceived}
              style={{ fontSize: '25px', marginLeft: '3%' }}
              value='right'
              variant='primary'
            >
              <img
                style={{ width: '44px', height: 'auto' }}
                src='/other_emoji_imgs/rightArrow.svg'
                alt='Right_Arrow_img'
                className='center'
              />
            </Button>
          </div>

          <div className='w-100'></div>
          <div className='col'>
            <Button
              onClick={cartesianControlCommandReceived}
              style={{ fontSize: '25px', marginLeft: '30%', marginRight: '15%' }}
              value='back'
              variant='primary'
            >
              <img
                style={{ width: '44px', height: 'auto' }}
                src='/other_emoji_imgs/downArrow.svg'
                alt='Down_Arrow_img'
                className='center'
              />
            </Button>
          </div>
          <div className='col'>
            <Button
              variant='success'
              onClick={doneClicked}
              style={{ width: '96%', fontSize: '25px', marginLeft: '10%', marginRight: '2%' }}
            >
              <img style={{ width: '30px', height: 'auto' }} src="/other_emoji_imgs/done.svg" alt='done_icon' /> Done
            </Button>
          </div>
        </div>
      </div>
    </div>
  )
}
PlateLocator.propTypes = {
  // The URL of the web video server
  webVideoServerURL: PropTypes.string.isRequired
}

export default PlateLocator
