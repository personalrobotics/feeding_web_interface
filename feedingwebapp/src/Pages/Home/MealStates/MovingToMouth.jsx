// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'
import Row from 'react-bootstrap/Row'

// Local Imports
import Footer from '../../Footer/Footer'
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'

/**
 * The MovingToMouth component tells the user that the robot is currently
 * moving to their mouth. It waits for the robot to complete the motion before
 * moving to the next state.
 *
 * @params {object} props - contains any properties passed to this Component
 */
const MovingToMouth = (props) => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)

  /**
   * Callback function for when the robot has finished moving to the user's
   * mouth.
   */
  function movingtoMouthDone() {
    console.log('movingtoMouthDone')
    setMealState(MEAL_STATE.U_BiteDone)
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
    <>
      {/* TODO: Consider vertically centering this element */}
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <div>
          <h1 id={MEAL_STATE.R_MovingAbovePlate} className='waitingMsg'>
            Waiting for Robot to move to your mouth...
          </h1>
          {props.debug ? (
            <Button variant='secondary' className='justify-content-center mx-2 mb-2' size='lg' onClick={movingtoMouthDone}>
              Continue (Debug Mode)
            </Button>
          ) : (
            <></>
          )}
        </div>
        {/* Ask the user whether they want to move to above plate position */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '148%' }}>
          Cancel bite? Move to staging.
        </p>
        {/* Icon to move above plate */}
      </Row>
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <Button
          variant='info'
          className='justify-content-center mx-2 mb-2 btn-huge'
          size='lg'
          onClick={moveToStagingPosition}
          style={{ width: '200px', height: '130px' }}
        >
          <img src='/robot_state_imgs/move_to_staging_position.svg' alt='move_to_staging_image' />
        </Button>
      </Row>
      {/**
       * Display the footer with the Pause button.
       */}
      <Footer />
    </>
  )
}
MovingToMouth.propTypes = {
  debug: PropTypes.bool
}

export default MovingToMouth
