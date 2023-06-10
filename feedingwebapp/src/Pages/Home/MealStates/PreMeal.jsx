// React Imports
import React, { useCallback } from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'
import { View } from 'react-native'
import { useMediaQuery } from 'react-responsive'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'

/**
 * The PreMeal component appears before the meal starts, and gives the user a
 * Start Feeding button to initiate feeding. Further, PreMeal is the only meal
 * state as of now where users can modify the settings.
 */
const PreMeal = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Font size for text
  let textFontSize = isPortrait ? '3vh' : '3vw'
  // Font size for button
  let buttonFontSize = isPortrait ? '5vh' : '5vw'
  // Width for button
  let buttonWidth = isPortrait ? '40vh' : '40vw'
  // Height for button
  let buttonHeight = isPortrait ? '10vh' : '10vw'
  // Margin
  let margin = isPortrait ? '5vh' : '5vw'

  /**
   * Callback function for when the user decides to start feeding using the app.
   */
  const startFeedingClicked = useCallback(() => {
    console.log('startFeedingClicked')
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }, [setMealState])

  // Render the component
  return (
    <View style={{ flex: 1, justifyContent: 'center', alignItems: 'center' }}>
      <Row xs={1} md={1} className='justify-content-center mx-2 my-2'>
        <p className='transitionMessage' style={{ marginBottom: margin, marginTop: '0', fontSize: textFontSize }}>
          Hello!👋 I am ADA&apos;s faithful assistant, ADAWebapp! Bon Appétit! 😋
        </p>
        <Button
          variant='primary'
          size='lg'
          className='btn-huge'
          id='#startFeedingBtn'
          onClick={startFeedingClicked}
          style={{ width: buttonWidth, height: buttonHeight, fontSize: buttonFontSize, marginTop: margin }}
        >
          Start Feeding
        </Button>
      </Row>
    </View>
  )
}

export default PreMeal
