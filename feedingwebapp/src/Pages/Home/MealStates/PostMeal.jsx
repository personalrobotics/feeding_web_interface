// React Imports
import React, { useCallback } from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'
import { useMediaQuery } from 'react-responsive'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'

/**
 * The PostMeal component appearsto the end of a meal, and lets the robot return
 * to the main menu.
 */
const PostMeal = () => {
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
   * Callback function for when the user decides to return to the start of the app.
   */
  const returnToStartClicked = useCallback(() => {
    console.log('startFeedingClicked')
    setMealState(MEAL_STATE.U_PreMeal)
  }, [setMealState])

  // Render the component
  return (
    <Row xs={1} md={1} className='justify-content-center mx-2 my-2'>
      <p className='transitionMessage' style={{ marginBottom: margin, marginTop: '0px', fontSize: textFontSize }}>
        That was a delicious meal 😋 ! Cheers 🥂 to your good health!
      </p>
      <Button
        variant='primary'
        size='lg'
        className='btn-huge'
        id='#returnToStartBtn'
        onClick={returnToStartClicked}
        style={{ width: buttonWidth, height: buttonHeight, fontSize: buttonFontSize, marginTop: margin }}
      >
        Return to Start
      </Button>
    </Row>
  )
}

export default PostMeal
