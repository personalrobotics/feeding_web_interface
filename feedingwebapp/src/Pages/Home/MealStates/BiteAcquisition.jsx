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
import plate from '../../../plate.svg'

/**
 * The BiteAcquisition component tells the user that the robot is currently
 * acquiring the bite. It waits for the robot to complete the motion before
 * moving to the next state.
 *
 * @params {object} props - contains any properties passed to this Component
 */
const BiteAcquisition = (props) => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  const desiredFoodItem = useGlobalState((state) => state.desiredFoodItem)

  /**
   * Callback function for if the user decides to cancel the bite.
   */
  function cancelBite() {
    console.log('cancelBite')
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }

  /**
   * Callback function for when the bite acquisition is done.
   */
  function biteAcquisitionDone() {
    console.log('biteAcquisitionDone')
    setMealState(MEAL_STATE.U_BiteAcquisitionCheck)
  }

  // Render the component
  return (
    <>
      {/* TODO: Consider vertically centering this element */}
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <div>
          <h1 id={MEAL_STATE.R_MovingAbovePlate} className='waitingMsg'>
            Waiting for the robot to acquire the {desiredFoodItem.toLowerCase()}...
          </h1>
          {props.debug ? (
            <Button variant='secondary' className='justify-content-center mx-2 mb-2' size='lg' onClick={biteAcquisitionDone}>
              Continue (Debug Mode)
            </Button>
          ) : (
            <></>
          )}
        </div>
      </Row>
      {/* Give the user the option to move robot over plate*/}
      <Row className='justify-content-center mx-auto my-2 w-75'>
        <Button
          variant='light'
          className='justify-content-center mx-2 mb-2 btn-huge'
          size='lg'
          onClick={cancelBite}
          style={{ width: '250px', height: '200px' }}
        >
          <img src={plate} />
        </Button>
      </Row>
      {/**
       * Display the footer with the Pause button.
       */}
      <Footer />
    </>
  )
}
BiteAcquisition.propTypes = {
  debug: PropTypes.bool
}

export default BiteAcquisition
