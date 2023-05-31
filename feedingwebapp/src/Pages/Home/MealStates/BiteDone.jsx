// React Imports
import React, { useCallback } from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'
import { useMediaQuery } from 'react-responsive'
import { View } from 'react-native'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { MOVING_STATE_ICON_DICT } from '../../Constants'

/**
 * The BiteDone component appears after the robot has moved to the user's mouth,
 * and waits for the user to specify that they have finished the bite before
 * moving back to above plate.
 */
const BiteDone = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  // Get icon image for move above plate
  let moveAbovePlateImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingAbovePlate]
  // Get icon image for move to resting position
  let moveToRestingPositionImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingToRestingPosition]
  // Get current orientation
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })

  /**
   * Callback function for when the user wants to move above plate.
   */
  const moveAbovePlate = useCallback(() => {
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }, [setMealState])

  /**
   * Callback function for when the user wants to move to resting position.
   */
  const moveToRestingPosition = useCallback(() => {
    setMealState(MEAL_STATE.R_MovingToRestingPosition)
  }, [setMealState])

  /**
   * Get the bite finished text to render.
   *
   * @returns {JSX.Element} the bite finished text
   */
  let biteFinishedText = function () {
    return (
      <>
        {/* Ask the user whether they want to move to above plate position */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '139%' }}>
          Bite finished? Move above plate.
        </p>
      </>
    )
  }

  /**
   * Get the bite finished button to render.
   *
   * @returns {JSX.Element} the bite finished button
   */
  let biteFinishedButton = function () {
    return (
      <>
        {/* Icon to move above plate */}
        <Button
          variant='success'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={moveAbovePlate}
          style={{ width: '300px', height: '200px' }}
        >
          <img src={moveAbovePlateImage} alt='move_above_plate_image' className='center' />
        </Button>
      </>
    )
  }

  /**
   * Get the take another bite text to render.
   *
   * @returns {JSX.Element} the take another bite text
   */
  let takeAnotherBiteText = function () {
    return (
      <>
        {/* Ask the user whether they want to move to resting position */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '139%' }}>
          Take another bite? Move to resting position.
        </p>
      </>
    )
  }

  /**
   * Get the take another bite button to render.
   *
   * @returns {JSX.Element} the take another bite button
   */
  let takeAnotherBiteButton = function () {
    return (
      <>
        {/* Icon to move to resting position */}
        <Button
          variant='warning'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          onClick={moveToRestingPosition}
          style={{ width: '300px', height: '200px' }}
        >
          <img src={moveToRestingPositionImage} alt='move_to_resting_image' className='center' />
        </Button>
      </>
    )
  }

  // Render the component
  return (
    <>
      {isPortrait ? (
        <div style={{ display: 'block', width: '100%', height: '115vh', overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
          {biteFinishedText()}
          {/* Icon to move above plate */}
          <Row className='justify-content-center mx-auto mb-2 w-75'>{biteFinishedButton()}</Row>
          {/* Add empty space */}
          <div className='justify-content-center mx-auto my-3 row'>&nbsp;</div>
          <Row className='justify-content-center mx-auto mt-2'>
            {takeAnotherBiteText()}
            {takeAnotherBiteButton()}
          </Row>
        </div>
      ) : (
        <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
          <View style={{ flex: '1', alignItems: 'center', justifyContent: 'center' }}>
            {biteFinishedText()}
            {biteFinishedButton()}
          </View>
          <View style={{ flex: '1', alignItems: 'center', justifyContent: 'center' }}>
            {takeAnotherBiteText()}
            {takeAnotherBiteButton()}
          </View>
        </View>
      )}
    </>
  )
}

export default BiteDone
