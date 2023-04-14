// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { REALSENSE_WIDTH, REALSENSE_HEIGHT } from '../../Constants'
import { scaleWidthHeightToWindow } from '../../../helpers'

/**
 * The PlateLocator component appears if the user decides to adjust the position
 * of the fork above the plate before selecting a bite. This component enables
 * the user to teleoperate the robot with Cartesian Control until the plate
 * is satisfactorily in view.
 *
 * @params {object} props - contains any properties passed to this Component
 */
const PlateLocator = () => {
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
  let { width, height } = scaleWidthHeightToWindow(REALSENSE_WIDTH, REALSENSE_HEIGHT)

  // Render the component
  return (
    <div style={{ overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
      {/**
       * Display the live stream from the robot's camera.
       */}
      <iframe
        src={`http://localhost:8080/stream?topic=/camera/color/image_raw&default_transport=compressed&width=${
          Math.round(width) - 30 // TODO: remove hardcoded value of 30, give it a descriptive name
        }&height=${Math.round(height)}&quality=20`}
        frameBorder='0'
        allow='autoplay; encrypted-media'
        allowFullScreen
        title='video'
        style={{ width: width, height: height }}
      />

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
              ⬆️
            </Button>
          </div>

          <div className='w-100'></div>

          <div className='col-3'>
            <Button onClick={cartesianControlCommandReceived} style={{ fontSize: '25px', marginLeft: '2%' }} value='left' variant='primary'>
              ⬅️
            </Button>
          </div>

          <div className='col'>
            <Button
              onClick={cartesianControlCommandReceived}
              style={{ fontSize: '25px', marginLeft: '3%' }}
              value='right'
              variant='primary'
            >
              ➡️
            </Button>
          </div>

          <div className='w-100'></div>
          <div className='col'>
            <Button
              onClick={cartesianControlCommandReceived}
              style={{ fontSize: '25px', marginLeft: '30%', marginTop: '5%', marginRight: '15%' }}
              value='back'
              variant='primary'
            >
              ⬇️
            </Button>
          </div>
          <div className='col'>
            <Button
              variant='success'
              onClick={doneClicked}
              style={{ width: '96%', fontSize: '25px', marginLeft: '10%', marginRight: '2%' }}
            >
              ✅ Done
            </Button>
          </div>
        </div>
      </div>
    </div>
  )
}

export default PlateLocator
