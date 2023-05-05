// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'

/**
 * The BiteAcquisitionCheck component appears after the robot has attempted to
 * acquire a bite, and asks the user whether it succeeded at acquiring the bite.
 */
const BiteAcquisitionCheck = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)

  /**
   * Callback function for when the user indicates that the bite acquisition
   * succeeded.
   */
  function acquisitionSuccess() {
    console.log('acquisitionSuccess')
    setMealState(MEAL_STATE.R_MovingToStagingLocation)
  }

  /**
   * Callback function for when the user indicates that the bite acquisition
   * failed.
   */
  function acquisitionFailure() {
    console.log('acquisitionFailure')
    setMealState(MEAL_STATE.R_BiteAcquisition)
  }

  // Render the component
  return (
    <div style={{ display: 'block', width: '100%', height: '115vh', overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
      {/**
       * Ask the user whether the robot succeeded at picking up food or not.
       */}
      <p className='transitionMessage' style={{ marginBottom: '0px' }}>
        Is there food on the fork?
      </p>
      {/* Ask the user whether they want to move to "ready" position */}
      <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '150%' }}>
        Yes, proceed
      </p>
      {/* Icon to move to staging position */}
      <Row className='justify-content-center mx-auto mb-2 w-75'>
        <Button variant='success' onClick={acquisitionSuccess} style={{ width: '300px', height: '200px' }}>
          <img src='/robot_state_imgs/move_to_staging_position.svg' alt='move_to_staging_image' className='center' />
        </Button>
      </Row>
      {/* Add empty space */}
      <div className='justify-content-center mx-auto my-2 row'>&nbsp;</div>
      <Row className='justify-content-center mx-auto mt-2'>
        {/* Ask the user whether they want to try again */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '150%' }}>
          No, try again
        </p>
        {/* Icon to move above plate */}
        <Button
          variant='warning'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={acquisitionFailure}
          style={{ width: '300px', height: '200px' }}
        >
          <img src='/robot_state_imgs/move_to_bite_acquisition_position.svg' alt='bite_acquisition_move_image' className='center' />
        </Button>
      </Row>
    </div>
  )
}

export default BiteAcquisitionCheck
