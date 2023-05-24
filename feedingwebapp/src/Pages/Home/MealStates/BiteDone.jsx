// React Imports
import React, { useCallback } from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { FOOTER_STATE_ICON_DICT } from '../../Constants'

/**
 * The BiteDone component appears after the robot has moved to the user's mouth,
 * and waits for the user to specify that they have finished the bite before
 * moving back to above plate.
 */
const BiteDone = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  // Get icon image for move above plate
  let moveAbovePlateImage = FOOTER_STATE_ICON_DICT[MEAL_STATE.R_MovingAbovePlate]

  /**
   * Callback function for when the user wants to move above plate.
   */
  const moveAbovePlate = useCallback(() => {
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }, [setMealState])

  // Render the component
  return (
    <div style={{ display: 'block', width: '100%', height: '115vh', overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
      {/* Add empty space */}
      <div className='justify-content-center mx-auto mt-5 row'>&nbsp;</div>
      {/* Ask the user whether they want to move to above plate position */}
      <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '140%' }}>
        Bite finished? Move above plate.
      </p>
      {/* Icon to move above plate */}
      <Row className='justify-content-center mx-auto mt-2 w-75'>
        <Button
          variant='success'
          className='mx-2 mt-2 btn-huge'
          size='lg'
          onClick={moveAbovePlate}
          style={{ width: '300px', height: '200px' }}
        >
          <img src={moveAbovePlateImage} alt='move_above_plate_image' className='center' />
        </Button>
      </Row>
    </div>
  )
}

export default BiteDone
