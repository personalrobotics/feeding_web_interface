// React imports
import React, { useCallback } from 'react'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'

// Local imports
import './Home.css'
import { useGlobalState, MEAL_STATE } from '../GlobalState'
import BiteAcquisition from './MealStates/BiteAcquisition'
import BiteAcquisitionCheck from './MealStates/BiteAcquisitionCheck'
import BiteDone from './MealStates/BiteDone'
import BiteInitiation from './MealStates/BiteInitiation'
import BiteSelection from './MealStates/BiteSelection'
import MovingAbovePlate from './MealStates/MovingAbovePlate'
import MovingToMouth from './MealStates/MovingToMouth'
import MovingToStagingLocation from './MealStates/MovingToStagingLocation'
import PlateLocator from './MealStates/PlateLocator'
import PostMeal from './MealStates/PostMeal'
import PreMeal from './MealStates/PreMeal'
import StowingArm from './MealStates/StowingArm'
import { TIME_TO_RESET_MS } from '../Constants'

/**
 * The Home component displays the state of the meal, solicits user input as
 * needed, and communicates with the robot (TODO (amaln)).
 */
function Home(props) {
  // Get the meal state
  const mealState = useGlobalState((state) => state.mealState)
  const mealStateTransitionTime = useGlobalState((state) => state.mealStateTransitionTime)
  const setMealState = useGlobalState((state) => state.setMealState)

  /**
   * Determines what screen to render based on the meal state.
   *
   */
  const getComponentByMealState = useCallback(() => {
    console.log('getComponentByMealState', mealState, mealStateTransitionTime, props.debug)

    // Implement time-based transition of states. This is so that after the user
    // finishes a meal, when they start the next meal the app starts in PreMeal.
    if (Date.now() - mealStateTransitionTime >= TIME_TO_RESET_MS) {
      console.log('Reverting to PreMeal due to too much elapsed time in one state.')
      setMealState(MEAL_STATE.U_PreMeal)
    }

    switch (mealState) {
      case MEAL_STATE.U_PreMeal:
        return <PreMeal debug={props.debug} />
      case MEAL_STATE.R_MovingAbovePlate:
        return <MovingAbovePlate debug={props.debug} />
      case MEAL_STATE.U_BiteSelection:
        return <BiteSelection debug={props.debug} />
      case MEAL_STATE.U_PlateLocator:
        return <PlateLocator debug={props.debug} />
      case MEAL_STATE.R_BiteAcquisition:
        return <BiteAcquisition debug={props.debug} />
      case MEAL_STATE.U_BiteAcquisitionCheck:
        return <BiteAcquisitionCheck debug={props.debug} />
      case MEAL_STATE.R_MovingToStagingLocation:
        return <MovingToStagingLocation debug={props.debug} />
      case MEAL_STATE.U_BiteInitiation:
        return <BiteInitiation debug={props.debug} />
      case MEAL_STATE.R_MovingToMouth:
        return <MovingToMouth debug={props.debug} />
      case MEAL_STATE.U_BiteDone:
        return <BiteDone debug={props.debug} />
      case MEAL_STATE.R_StowingArm:
        return <StowingArm debug={props.debug} />
      case MEAL_STATE.U_PostMeal:
        return <PostMeal debug={props.debug} />
    }
  }, [mealState, mealStateTransitionTime, setMealState])

  // Render the component
  return (
    <div>
      {/**
       * The main contents of the screen depends on the mealState.
       */}
      {getComponentByMealState()}
    </div>
  )
}
Home.propTypes = {
  debug: PropTypes.bool
}

export default Home
