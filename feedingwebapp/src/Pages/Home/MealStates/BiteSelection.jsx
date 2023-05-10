// React Imports
import React from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'

// TODO: Replace the list of items with a view of the camera feed that the user
// can click on.
let food = ['Apple', 'Banana', 'Carrot', 'Cucumber']

/**
 * The BiteSelection component appears after the robot has moved above the plate,
 * plate. It enables users to select their desired food item.
 */
const BiteSelection = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  const setDesiredFoodItem = useGlobalState((state) => state.setDesiredFoodItem)

  /**
   * Callback function for when the user indicates that they want to move the
   * robot to locate the plate.
   */
  function locatePlateClicked() {
    console.log('locatePlateClicked')
    setMealState(MEAL_STATE.U_PlateLocator)
  }

  /**
   * Callback function for when the user wants to move to staging position.
   */
  function moveToStagingPosition() {
    setMealState(MEAL_STATE.R_MovingToStagingLocation)
  }

  /**
   * Callback function for when the user indicates that they are done with their
   * meal.
   */
  function doneEatingClicked() {
    console.log('doneEatingClicked')
    setMealState(MEAL_STATE.R_StowingArm)
  }

  /**
   * Callback function for when the user clicks the button for a food item.
   */
  function foodItemClicked(event) {
    let foodName = event.target.value
    console.log('foodItemClicked', foodName)
    setDesiredFoodItem(foodName)
    setMealState(MEAL_STATE.R_BiteAcquisition)
  }

  // Render the component
  return (
    <>
      {/**
       * In addition to selecting their desired food item, the user has two
       * other options on this page:
       *   - If their desired food item is not visible on the plate, they can
       *     decide to teleoperate the robot until it is visible.
       *   - Instead of selecting their next bite, the user can indicate that
       *     they are done eating.
       */}
      <div style={{ display: 'block', textAlign: 'center' }}>
        <Button className='doneButton' style={{ fontSize: '21px', marginTop: '10px' }} onClick={locatePlateClicked}>
          🍽️ Locate Plate
        </Button>
        <Button className='doneButton' style={{ fontSize: '21px', marginTop: '10px' }} onClick={doneEatingClicked}>
          ✅ Done Eating
        </Button>
      </div>
      {/**
       * Display a list of the detected food items.
       *
       * TODO: Replace this with an image of the plate where users can click on
       * their desired food item.
       */}
      <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '20px' }}>
        Choose from one of the following food items.
      </p>
      <Row xs={3} s={2} md={3} lg={4} className='justify-content-center mx-auto my-2' style={{ paddingBottom: '5vh' }}>
        {food.map((value, i) => (
          <Button
            key={i}
            variant='primary'
            className='mx-1 mb-1'
            style={{ paddingLeft: '0px', paddingRight: '0px', marginLeft: '0px', marginRight: '0px', fontSize: '22px' }}
            value={value}
            size='lg'
            onClick={foodItemClicked}
          >
            {value}
          </Button>
        ))}
      </Row>
      <div style={{ display: 'block', width: '100%', height: '115vh', overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
        {/* Ask the user whether they want to continue without acquisition by moving to above plate position */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '140%' }}>
          Continue without acquiring bite? Move to staging position.
        </p>
        {/* Icon to move above plate */}
        <Row className='justify-content-center mx-auto mb-2 w-75'>
          <Button
            variant='warning'
            className='mx-2 mb-2 btn-huge'
            size='lg'
            onClick={moveToStagingPosition}
            style={{ width: '300px', height: '200px' }}
          >
            <img src='/robot_state_imgs/move_to_staging_position.svg' alt='move_above_plate_image' className='center' />
          </Button>
        </Row>
      </div>
    </>
  )
}

export default BiteSelection
