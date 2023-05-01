// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import above_plate_position_img from '../robot_state_imgs/above_plate_position.svg'
import move_to_mouth_position_img from '../robot_state_imgs/move_to_mouth_position.svg'

/**
 * The BiteInitiation component appears after the robot has moved to the staging
 * position, and waits for the user to indicate that they are ready for a bite.
 */
const BiteInitiation = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)

  /**
   * Callback function for when the user is ready for their bite.
   */
  function readyForBite() {
    setMealState(MEAL_STATE.R_MovingToMouth)
  }

  /**
   * Callback function for when the user wants to move above plate.
   */
  function moveAbovePlate() {
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }

  // Render the component
  return (
    <div style={{ display: 'block', width: '100%', height: '115vh', overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
      {/* Ask the user whether they want to move to mouth position */}
      <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '148%' }}>
        Move to mouth position.
      </p>
      {/* Icon to move to mouth */}
      {/* Ask the user whether they're ready for the bite */}
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <Button
          variant='success'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={readyForBite}
          style={{ width: '200px', height: '130px' }}
        >
          <img src={move_to_mouth_position_img} />
        </Button>
      </Row>

      {/* Ask the user whether they want to move to above plate position */}
      <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '148%' }}>
        Move to above plate position.
      </p>
      {/* Icon to move above plate */}
      {/* Give the user the option to move robot over plate*/}
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
    </div>
  )
}

export default BiteInitiation
