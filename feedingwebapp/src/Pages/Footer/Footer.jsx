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
  /**
   * Regardless of the state, the back button should revert to MoveAbovePlate.
   *   - BiteAcquisition: In this case, pressing "back" should let the user
   *     reselect the bite, which requires the robot to move above plate.
   *   - MoveToStagingLocation: In this case, pressing "back" should move the
   *     robot back to the plate. Although the user may not always want to
   *     reselect the bite, from `BiteSelection` they have the option to skip
   *     BiteAcquisition and move straight to staging location (when they are ready).
   *   - MoveToMouth: Although in some cases the user may want "back" to move to
   *     the staging location, since we will be removing the staging location
   *     (Issue #45) it makes most sense to move the robot back to the plate.
   *   - StowingArm: In this case, if the user presses back they likely want to
   *     eat another bite, hence moving above the plate makes sense.
   *   - MovingAbovePlate: Although the user may want to press "back" to move
   *     the robot to the staging location, they can also go forward to
   *     BiteSelection and then move the robot to the staging location.
   *     Hence, in this case we don't have a "back" button.
   */
  // A local variable for storing back icon image
  var backIcon = FOOTER_STATE_ICON_DICT[MEAL_STATE.R_MovingAbovePlate]
  // A local variable for storing current state icon image
  var resumeIcon = FOOTER_STATE_ICON_DICT[mealState]
  // Local state variable to track of visibility of pause button
  const [pauseButtonVisible, setPauseButtonVisible] = useState(true)
  // width of Back and Resume buttons
  let backResumeButtonWidth = '150px'
  let footerButtonHight = '100px'

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

  /**
   * Get the footer text and buttons to render.
   *
   * @returns {JSX.Element} the footer text and buttons
   */
  let footerTextAndButtons = function () {
    /**
     * If the pause button is visible, regardless of the meal state
     * show a pause button taking the whole width of the footer screen.
     */
    if (pauseButtonVisible) {
      return (
        <>
          <Row className='justify-content-center mx-auto'>
            <p className='transitionMessage' style={{ marginBottom: '0', fontSize: '170%', color: 'white', fontWeight: 'bold' }}>
              ⏸️ Pause
            </p>
            {/* Icon to pause */}
            <Button
              variant='danger'
              onClick={() => setPauseButtonVisible(false)}
              style={{ marginLeft: '10', marginRight: '10', marginTop: '0', width: '350px', height: { footerButtonHight } }}
            >
              <img
                style={{ width: '135px', height: '90px' }}
                src='/robot_state_imgs/pause_button_icon.svg'
                alt='pause_icon_img'
                className='center'
              />
            </Button>
          </Row>
        </>
      )
    } else {
      /** If the robot is moving above the plate and pause button is not visible,
       *  only show the resume button and no back button, since the user
       * can go "forward" to any other state as opposed to going back.
       */
      if (mealState === MEAL_STATE.R_MovingAbovePlate) {
        return (
          <>
            <View style={{ flexDirection: 'row' }}>
              <View style={{ width: { backResumeButtonWidth } }}></View>
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
                  style={{ marginLeft: 190, marginRight: 10, width: { backResumeButtonWidth }, height: { footerButtonHight } }}
                >
                  <img style={{ width: '120px', height: '72px' }} src={resumeIcon} alt='resume_icon_img' className='center' />
                </Button>
              </View>
            </View>
          </>
        )
      } else if (mealState === MEAL_STATE.R_BiteAcquisition) {
        /** If the robot is aquiring the bite and pause button is not visible,
         * only show the back button and no resume button, since the user
         * can continue to acquiring bite again after moving above plate,
         * but if they resume this state after aquiring bite, bite selection
         * mask may no longer work because the food may have shifted
         */
        return (
          <>
            <View style={{ flexDirection: 'row' }}>
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
                  style={{ marginLeft: 10, marginRight: 10, width: { backResumeButtonWidth }, height: { footerButtonHight } }}
                >
                  <img style={{ width: '120px', height: '72px' }} src={backIcon} alt='back_icon_img' className='center' />
                </Button>
              </View>
              <View style={{ width: { backResumeButtonWidth } }}></View>
            </View>
          </>
        )
      } else {
        /** For any other meal state (e.g., MoveToStagingLocation, MoveToMouth,
         * and StowingArm), if the pause button is not visible, then the footer
         * shows both the back and resume buttons
         */
        return (
          <>
            <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
              <View>
                <p className='transitionMessage' style={{ marginBottom: '0', fontSize: '170%', color: 'white', fontWeight: 'bold' }}>
                  ◀️ Back
                </p>
                {/* Icon to move to previous state */}
                <Button
                  variant='warning'
                  onClick={backButtonClicked}
                  style={{ marginLeft: 10, marginRight: 10, width: { backResumeButtonWidth }, height: { footerButtonHight } }}
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
                  style={{ marginLeft: 10, marginRight: 10, width: { backResumeButtonWidth }, height: { footerButtonHight } }}
                >
                  <img style={{ width: '120px', height: '72px' }} src={resumeIcon} alt='resume_icon_img' className='center' />
                </Button>
              </View>
            </View>
          </>
        )
      }
    }
  }

  // Render the component
  return (
    <>
      {/**
       * The footer shows a pause button first. A resume button and a back button are shown when the pause button is clicked.
       */}
      <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
        <div className='text-center p-3' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)' }}>
          {footerTextAndButtons()}
        </div>
      </MDBFooter>
    </>
  )
}

export default Footer
