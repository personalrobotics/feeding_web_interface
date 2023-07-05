// React Imports
import React, { useCallback } from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'

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

  // Icons for rendering
  let wave = '/other_emoji_imgs/wave.svg'
  let yummy = '/other_emoji_imgs/yummy.svg'

  /**
   * Callback function for when the user decides to start feeding using the app.
   */
  const startFeedingClicked = useCallback(() => {
    console.log('startFeedingClicked')
    setMealState(MEAL_STATE.R_MovingAbovePlate)
  }, [setMealState])

  // Render the component
  return (
    <Row xs={1} md={1} className='justify-content-center mx-2 my-2'>
      <p className='transitionMessage' style={{ marginBottom: '10px', marginTop: '0px', fontSize: '24px' }}>
        Hello! <img style={{ width: '30px', height: 'auto' }} src={wave} alt='wave_icon' /> I am ADA&apos;s faithful assistant, ADAWebapp! Bon Appétit! <img style={{ width: '30px', height: 'auto' }} src={yummy} alt='yummy_emoji_icon'/>
      </p>
      <Button
        variant='primary'
        size='lg'
        className='btn-huge'
        id='#startFeedingBtn'
        onClick={startFeedingClicked}
        style={{ width: '75%', fontSize: '35px', marginTop: '30px' }}
      >
        Start Feeding
      </Button>
    </Row>
  )
}

export default PreMeal
