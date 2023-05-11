// React Imports
import React, {useState} from 'react'
import Button from 'react-bootstrap/Button'

// Local Imports
import '../Home.css'
import ImageWithButton from './ImageWithButtonName'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'

/**
 * The BiteSelection component appears after the robot has moved above the plate,
 * plate. It enables users to select their desired food item.
 */
const BiteSelectionName = () => {
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
   * Callback function for when the user indicates that they are done with their
   * meal.
   */
  function doneEatingClicked() {
    console.log('doneEatingClicked')
    setMealState(MEAL_STATE.R_StowingArm)
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
      <div style={{ display: 'block' }}>
        <Button
          className='doneButton'
          style={{ fontSize: '24px', marginTop: '0px', marginRight: '10px', marginLeft: 'auto', display: 'block' }}
          onClick={locatePlateClicked}
          disabled={true}
        >
          🍽️ Locate Plate
        </Button>
        <Button
          className='doneButton'
          style={{ fontSize: '24px', marginTop: '0px', marginRight: '10px', marginLeft: 'auto', display: 'block' }}
          onClick={doneEatingClicked}
          disabled={true}
        >
          ✅ Done Eating
        </Button>
      </div>
      <div>
        <ImageWithButton
          imgSrc='https://www.diabetesfoodhub.org/system/user_files/Images/1837-diabetic-pecan-crusted-chicken-breast_JulAug20DF_clean-simple_061720.jpg'
          imgWidth={1021}
          imgHeight={779}
          foodItems={["Chicken", "Salad", "Fries"]}
        />
      </div>
    </>
  )
}

export default BiteSelectionName
