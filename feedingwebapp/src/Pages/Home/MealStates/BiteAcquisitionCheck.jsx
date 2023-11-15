// React Imports
import React, { useCallback } from 'react'
import Button from 'react-bootstrap/Button'
import { useMediaQuery } from 'react-responsive'
import { View } from 'react-native'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { MOVING_STATE_ICON_DICT } from '../../Constants'

/**
 * The BiteAcquisitionCheck component appears after the robot has attempted to
 * acquire a bite, and asks the user whether it succeeded at acquiring the bite.
 */
const BiteAcquisitionCheck = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  // Get icon image for move above plate
  let moveAbovePlateImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingAbovePlate]
  // Get icon image for move to mouth
  let moveToStagingConfigurationImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingToStagingConfiguration]
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

  /**
   * Callback function for when the user indicates that the bite acquisition
   * succeeded.
   */
  const acquisitionSuccess = useCallback(() => {
    console.log('acquisitionSuccess')
    setMealState(MEAL_STATE.R_MovingToStagingConfiguration)
  }, [setMealState])

  /**
   * Callback function for when the user indicates that the bite acquisition
   * failed.
   */
  const acquisitionFailure = useCallback(() => {
    console.log('acquisitionFailure')
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }, [setMealState])

  /**
   * Get the ready for bite text to render.
   *
   * @returns {JSX.Element} the ready for bite text
   */
  const readyForBiteText = useCallback(() => {
    return (
      <>
        {/* Ask the user whether they want to move to mouth position */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
          Ready for bite? Move to mouth.
        </p>
      </>
    )
  }, [textFontSize])

  /**
   * Get the ready for bite button to render.
   *
   * @returns {JSX.Element} the ready for bite button
   */
  const readyForBiteButton = useCallback(() => {
    return (
      <>
        {/* Icon to move to mouth position */}
        <Button
          variant='success'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={acquisitionSuccess}
          style={{ width: buttonWidth, height: buttonHeight }}
        >
          <img
            src={moveToStagingConfigurationImage}
            alt='move_to_mouth_image'
            className='center'
            style={{ width: iconWidth, height: iconHeight }}
          />
        </Button>
      </>
    )
  }, [moveToStagingConfigurationImage, acquisitionSuccess, buttonHeight, buttonWidth, iconHeight, iconWidth])

  /**
   * Get the re-acquire bite text to render.
   *
   * @returns {JSX.Element} the re-acquire bite text
   */
  const reacquireBiteText = useCallback(() => {
    return (
      <>
        {/* Ask the user whether they want to try acquiring bite again */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
          Re-acquire bite? Move above plate.
        </p>
      </>
    )
  }, [textFontSize])

  /**
   * Get the re-acquire bite button to render.
   *
   * @returns {JSX.Element} the re-acquire bite button
   */
  const reacquireBiteButton = useCallback(() => {
    return (
      <>
        {/* Icon for move above plate */}
        <Button
          variant='warning'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={acquisitionFailure}
          style={{ width: buttonWidth, height: buttonHeight }}
        >
          <img src={moveAbovePlateImage} alt='move_above_plate_image' className='center' style={{ width: iconWidth, height: iconHeight }} />
        </Button>
      </>
    )
  }, [acquisitionFailure, moveAbovePlateImage, buttonHeight, buttonWidth, iconHeight, iconWidth])

  /** Get the full page view
   *
   * @returns {JSX.Element} the the full page view
   */
  const fullPageView = useCallback(() => {
    return (
      <View style={{ flex: 'auto', flexDirection: dimension, alignItems: 'center', justifyContent: 'center' }}>
        <View style={{ flex: 5, alignItems: 'center', justifyContent: 'center' }}>
          {readyForBiteText()}
          {readyForBiteButton()}
        </View>
        <View style={{ flex: 5, alignItems: 'center', justifyContent: 'center' }}>
          {reacquireBiteText()}
          {reacquireBiteButton()}
        </View>
      </View>
    )
  }, [dimension, reacquireBiteButton, reacquireBiteText, readyForBiteButton, readyForBiteText])

  // Render the component
  return <>{fullPageView()}</>
}

export default BiteAcquisitionCheck
