// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import plate from '../../../plate.svg'
import mouth from '../../../mouth.svg'

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
    console.log('readyForBite')
    setMealState(MEAL_STATE.R_MovingToMouth)
  }

  /**
   * Callback function for if the user decides to cancel the bite.
   *
   * TODO: Think more carefully about what cancelBite at this stage should do!
   * Maybe replace with a more descriptive button (e.g., "return above plate.")
   */
  function cancelBite() {
    console.log('cancelBite')
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }

  // Render the component
  return (
    <div style={{ display: 'block', width: '100%', height: '115vh', overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
      {/* Ask the user whether they're ready for the bite */}
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <Button variant='light' className='mx-2 mb-2 btn-huge' size='lg' onClick={readyForBite} style={{ width: '250px', height: '200px' }}>
          <img src={mouth} />
        </Button>
      </Row>

      {/* Give the user the option to move robot over plate*/}
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <Button variant='light' className='mx-2 mb-2 btn-huge' size='lg' onClick={cancelBite} style={{ width: '250px', height: '200px' }}>
          <img src={plate} />
        </Button>
      </Row>
    </div>
  )
}

export default BiteInitiation
