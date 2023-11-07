// React Imports
import React, { useCallback, useEffect, useRef } from 'react'
import Button from 'react-bootstrap/Button'
import { useMediaQuery } from 'react-responsive'
import { View } from 'react-native'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import {
  FOOD_ON_FORK_TOPIC,
  FOOD_ON_FORK_PROB_RANGE,
  FOOD_ON_FORK_BITE_TRANSFER_WINDOW_SIZE,
  MOVING_STATE_ICON_DICT
} from '../../Constants'

// Import subscriber to be able to subscribe to FoF topic
import { subscribeToROSTopic, unsubscribeFromROSTopic, useROS } from '../../../ros/ros_helpers'

/**
 * The BiteDone component appears after the robot has moved to the user's mouth,
 * and waits for the user to specify that they have finished the bite before
 * moving back to above plate.
 */
const BiteDone = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  const foodOnFork = useGlobalState((state) => state.foodOnFork)
  // Get icon image for move above plate
  let moveAbovePlateImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingFromMouthToAbovePlate]
  // Get icon image for move to resting position
  let moveToRestingPositionImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingFromMouthToRestingPosition]
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Indicator of how to arrange screen elements based on orientation
  let dimension = isPortrait ? 'column' : 'row'
  // Font size for text
  let textFontSize = isPortrait ? '3vh' : '3vw'
  let buttonWidth = isPortrait ? '30vh' : '30vw'
  let buttonHeight = isPortrait ? '20vh' : '20vw'
  let iconWidth = isPortrait ? '28vh' : '28vw'
  let iconHeight = isPortrait ? '18vh' : '18vw'

  // Connect to Ros
  const ros = useRef(useROS().ros)
  let window = []
  const food_on_fork_callback = useCallback((message) => {
    console.log('Subscribed to FoF')
    if (window.length === Number(FOOD_ON_FORK_BITE_TRANSFER_WINDOW_SIZE)) {
      window.shift()
    }
    window.push(Number(message.data))
    let countLessThanRange = 0
    for (const val of window) {
      if (val < FOOD_ON_FORK_PROB_RANGE.lowerProb) {
        countLessThanRange++
      }
    }
    console.log(window.length)
    if (window.length === FOOD_ON_FORK_BITE_TRANSFER_WINDOW_SIZE && countLessThanRange >= 0.75 * FOOD_ON_FORK_BITE_TRANSFER_WINDOW_SIZE) {
      if (foodOnFork === "Yes") {
        console.log('Detecting no food on fork; moving above plate')
        moveAbovePlate()
      }
      return
    }
  })

  useEffect(() => {
    const food_on_fork_topic = subscribeToROSTopic(ros.current, FOOD_ON_FORK_TOPIC.name, FOOD_ON_FORK_TOPIC.type, food_on_fork_callback)

    return () => {
      console.log('unscubscribed from FoF')
      unsubscribeFromROSTopic(food_on_fork_topic)
    }
  }, [setMealState, food_on_fork_callback, foodOnFork])

  /**
   * Callback function for when the user wants to move above plate.
   */
  const moveAbovePlate = useCallback(() => {
    setMealState(MEAL_STATE.R_MovingFromMouthToAbovePlate)
  }, [setMealState])

  /**
   * Callback function for when the user wants to move to resting position.
   */
  const moveToRestingPosition = useCallback(() => {
    setMealState(MEAL_STATE.R_MovingFromMouthToRestingPosition)
  }, [setMealState])

  /**
   * Get the bite finished text to render.
   *
   * @returns {JSX.Element} the bite finished text
   */
  const biteFinishedText = useCallback(() => {
    return (
      <>
        {/* Ask the user whether they want to move to above plate position */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
          Bite finished? Move above plate.
        </p>
      </>
    )
  }, [textFontSize])

  /**
   * Get the bite finished button to render.
   *
   * @returns {JSX.Element} the bite finished button
   */
  const biteFinishedButton = useCallback(() => {
    return (
      <>
        {/* Icon to move above plate */}
        <Button
          variant='success'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={moveAbovePlate}
          style={{ width: buttonWidth, height: buttonHeight }}
        >
          <img src={moveAbovePlateImage} alt='move_above_plate_image' className='center' style={{ width: iconWidth, height: iconHeight }} />
        </Button>
      </>
    )
  }, [moveAbovePlate, moveAbovePlateImage, buttonHeight, buttonWidth, iconHeight, iconWidth])

  /**
   * Get the take another bite text to render.
   *
   * @returns {JSX.Element} the take another bite text
   */
  const takeAnotherBiteText = useCallback(() => {
    return (
      <>
        {/* Ask the user whether they want to move to resting position */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
          Take another bite? Move back.
        </p>
      </>
    )
  }, [textFontSize])

  /**
   * Get the take another bite button to render.
   *
   * @returns {JSX.Element} the take another bite button
   */
  const takeAnotherBiteButton = useCallback(() => {
    return (
      <>
        {/* Icon to move to resting position */}
        <Button
          variant='warning'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={moveToRestingPosition}
          style={{ width: buttonWidth, height: buttonHeight }}
        >
          <img
            src={moveToRestingPositionImage}
            alt='move_to_resting_image'
            className='center'
            style={{ width: iconWidth, height: iconHeight }}
          />
        </Button>
      </>
    )
  }, [moveToRestingPosition, moveToRestingPositionImage, buttonHeight, buttonWidth, iconHeight, iconWidth])

  /** Get the full page view
   *
   * @returns {JSX.Element} the the full page view
   */
  const fullPageView = useCallback(() => {
    return (
      <View style={{ flex: 'auto', flexDirection: dimension, alignItems: 'center', justifyContent: 'center', width: '100%' }}>
        <View style={{ flex: 5, alignItems: 'center', justifyContent: 'center' }}>
          {biteFinishedText()}
          {biteFinishedButton()}
        </View>
        <View style={{ flex: 5, alignItems: 'center', justifyContent: 'center' }}>
          {takeAnotherBiteText()}
          {takeAnotherBiteButton()}
        </View>
      </View>
    )
  }, [dimension, biteFinishedButton, biteFinishedText, takeAnotherBiteButton, takeAnotherBiteText])

  // Render the component
  return <>{fullPageView()}</>
}

export default BiteDone
