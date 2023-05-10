// React imports
import React, { useState } from 'react'
import { MDBFooter } from 'mdb-react-ui-kit'
import Button from 'react-bootstrap/Button'
import { View } from 'react-native'
import Row from 'react-bootstrap/Row'

// Local imports
import { FOOTER_STATE_ICON_DICT } from '../Constants'
import { useGlobalState, MEAL_STATE } from '../GlobalState'

/**
 * The Footer shows a pause button. When users click it, the app tells the robot
 * to immediately pause and displays a back button that allows them to return to
 * previous state and a resume button that allows them to resume current state.
 */
const Footer = () => {
  // Set the current meal state
  const setMealState = useGlobalState((state) => state.setMealState)
  // Get the current meal state
  const mealState = useGlobalState((state) => state.mealState)
  // A local variable for storing back icon image
  var backIcon = FOOTER_STATE_ICON_DICT[MEAL_STATE.R_MovingAbovePlate]
  // A local variable for storing current state icon image
  var resumeIcon = FOOTER_STATE_ICON_DICT[mealState]
  // Local state variable to track of visibility of pause button
  const [pauseButtonVisible, setPauseButtonVisible] = useState(true)

  /**
   * When the back button is clicked, go back to previous state.
   */
  function backButtonClicked() {
    // Set meal state to move above plate
    setMealState(MEAL_STATE.R_MovingAbovePlate)
    // We call setPauseButtonVisible with a new value to make pause button visible again.
    // React will re-render the Footer component.
    setPauseButtonVisible(true)
  }

  /**
   * When the resume button is clicked, continue with the current meal state.
   */
  function resumeButtonClicked() {
    // We call setPauseButtonVisible with a new value to make pause button visible again.
    // React will re-render the Footer component.
    setPauseButtonVisible(true)
  }

  return (
    <>
      {/**
       * The footer shows a pause button first. A resume button and a back button are shown when the pause button is clicked.
       */}
      <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
        <div className='text-center p-3' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)' }}>
          {pauseButtonVisible ? (
            <Row className='justify-content-center mx-auto'>
              <p className='transitionMessage' style={{ marginBottom: '0', fontSize: '170%', color: 'white', fontWeight: 'bold' }}>
                ⏸️ Pause
              </p>
              {/* Icon to pause */}
              <Button
                variant='danger'
                onClick={() => setPauseButtonVisible(false)}
                style={{ marginLeft: '10', marginRight: '10', marginTop: '0', width: '350px', height: '100px' }}
              >
                <img
                  style={{ width: '135px', height: '90px' }}
                  src='/robot_state_imgs/pause_button_icon.svg'
                  alt='pause_icon_img'
                  className='center'
                />
              </Button>
            </Row>
          ) : mealState === MEAL_STATE.R_MovingAbovePlate ? (
            <View>
              <p
                className='transitionMessage'
                style={{ marginBottom: '0', fontSize: '170%', color: 'white', fontWeight: 'bold', textAlign: 'right' }}
              >
                ▶️ Resume
              </p>
              {/* Icon to resume current state */}
              <Button
                variant='success'
                onClick={resumeButtonClicked}
                style={{ marginLeft: 225, marginRight: 10, width: '150px', height: '100px' }}
              >
                <img style={{ width: '120px', height: '72px' }} src={resumeIcon} alt='resume_icon_img' className='center' />
              </Button>
            </View>
          ) : mealState === MEAL_STATE.R_BiteAcquisition ? (
            <View>
              <p
                className='transitionMessage'
                style={{ marginBottom: '0', fontSize: '170%', color: 'white', fontWeight: 'bold', textAlign: 'left' }}
              >
                ◀️ Back
              </p>
              {/* Icon to move to previous state */}
              <Button
                variant='warning'
                onClick={backButtonClicked}
                style={{ marginLeft: 10, marginRight: 10, width: '150px', height: '100px' }}
              >
                <img style={{ width: '120px', height: '72px' }} src={backIcon} alt='back_icon_img' className='center' />
              </Button>
            </View>
          ) : (
            <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
              <View>
                <p className='transitionMessage' style={{ marginBottom: '0', fontSize: '170%', color: 'white', fontWeight: 'bold' }}>
                  ◀️ Back
                </p>
                {/* Icon to move to previous state */}
                <Button
                  variant='warning'
                  onClick={backButtonClicked}
                  style={{ marginLeft: 10, marginRight: 10, width: '150px', height: '100px' }}
                >
                  <img style={{ width: '120px', height: '72px' }} src={backIcon} alt='back_icon_img' className='center' />
                </Button>
              </View>
              <View>
                <p className='transitionMessage' style={{ marginBottom: '0', fontSize: '170%', color: 'white', fontWeight: 'bold' }}>
                  ▶️ Resume
                </p>
                {/* Icon to resume */}
                <Button
                  variant='success'
                  onClick={resumeButtonClicked}
                  style={{ marginLeft: 10, marginRight: 10, width: '150px', height: '100px' }}
                >
                  <img style={{ width: '120px', height: '72px' }} src={resumeIcon} alt='resume_icon_img' className='center' />
                </Button>
              </View>
            </View>
          )}
        </div>
      </MDBFooter>
    </>
  )
}

export default Footer
