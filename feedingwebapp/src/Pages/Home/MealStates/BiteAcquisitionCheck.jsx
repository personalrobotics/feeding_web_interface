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
 * The BiteAcquisitionCheck component appears after the robot has attempted to
 * acquire a bite, and asks the user whether it succeeded at acquiring the bite.
 */
const BiteAcquisitionCheck = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  // Get icon image for move above plate
  let moveAbovePlateImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingAbovePlate]
  // Get icon image for move to mouth
  let moveToMouthImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingToMouth]
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })

  /**
   * Callback function for when the user indicates that the bite acquisition
   * succeeded.
   */
  const acquisitionSuccess = useCallback(() => {
    console.log('acquisitionSuccess')
    setMealState(MEAL_STATE.R_MovingToMouth)
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
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '140%' }}>
          Ready for bite? Move to mouth.
        </p>
      </>
    )
  }, [])

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
          style={{ width: '300px', height: '200px' }}
        >
          <img src={moveToMouthImage} alt='move_to_mouth_image' className='center' />
        </Button>
      </>
    )
  }, [moveToMouthImage, acquisitionSuccess])

  /**
   * Get the re-acquire bite text to render.
   *
   * @returns {JSX.Element} the re-acquire bite text
   */
  const reacquireBiteText = useCallback(() => {
    return (
      <>
        {/* Ask the user whether they want to try acquiring bite again */}
        <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: '140%' }}>
          Re-acquire bite? Move above plate.
        </p>
      </>
    )
  }, [])

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
          style={{ width: '300px', height: '200px' }}
        >
          <img src={moveAbovePlateImage} alt='move_above_plate_image' className='center' />
        </Button>
      </>
    )
  }, [acquisitionFailure, moveAbovePlateImage])

  // Render the component
  return (
    <>
      {isPortrait ? (
        <div style={{ display: 'block', width: '100%', height: '115vh', overflowX: 'hidden', overflowY: 'auto' }} className='outer'>
          {readyForBiteText()}
          <Row className='justify-content-center mx-auto w-75'>{readyForBiteButton()}</Row>
          {/* Add empty space */}
          <div className='justify-content-center mx-auto mb-1 row'>&nbsp;</div>
          <Row className='justify-content-center mx-auto mb-2'>
            {reacquireBiteText()}
            {reacquireBiteButton()}
          </Row>
        </div>
      ) : (
        <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
          <View style={{ flex: '1', alignItems: 'center', justifyContent: 'center' }}>
            {readyForBiteText()}
            {readyForBiteButton()}
          </View>
          <View style={{ flex: '1', alignItems: 'center', justifyContent: 'center' }}>
            {reacquireBiteText()}
            {reacquireBiteButton()}
          </View>
        </View>
      )}
    </>
  )
}

export default BiteAcquisitionCheck
