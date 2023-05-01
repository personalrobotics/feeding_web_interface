// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import move_to_staging_position_img from '../robot_state_imgs/move_to_staging_position.svg'
import above_plate_position_img from '../robot_state_imgs/above_plate_position.svg'

/**
 * The BiteDone component appears after the robot has moved to the user's mouth,
 * and waits for the user to specify that they have finished the bite before
 * moving back to the staging area.
 */
const BiteDone = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)

  /**
   * Callback function for when the user wants to move above plate.
   */
  function moveAbovePlate() {
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }

  /**
   * Callback function for if the user decides to move to staging position.
   *
   */
  function moveToStagingPosition() {
    setMealState(MEAL_STATE.R_MovingToStagingLocation)
  }

  // Render the component
  return (
    <div style={{ display: 'block', width: '100%', height: '115vh', overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
      {/* Ask the user whether they want to move to above plate position */}
      <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '148%' }}>
        Move to above plate position.
      </p>
      {/* Icon to move above plate */}
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <Button
          variant='danger'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={moveAbovePlate}
          style={{ width: '200px', height: '130px' }}
        >
          <img src={above_plate_position_img} />
        </Button>
      </Row>

      {/* Ask the user whether they want to move to staging position */}
      <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '148%' }}>
        Move to staging position.
      </p>
      {/* Icon to move to staging position */}
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <Button
          variant='warning'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={moveToStagingPosition}
          style={{ width: '200px', height: '130px' }}
        >
          <img src={move_to_staging_position_img} />
        </Button>
      </Row>
    </div>
  )
}

export default BiteDone
