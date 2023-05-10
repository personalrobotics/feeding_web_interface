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
  /** The backIcon is used in all meal states where the robot moves (except the
   * MoveAbovePlate state) which include BiteAcquisition, MoveToStagingLocation,
   * MoveToMouth, and StowingArm meal states. For the reasons why the users may
   * press “Back” in these states can potentially entail acquiring wrong item,
   * food falling off the fork, changing mind for being done with eating or food
   * item choice, waiting in the staging location, and avoiding accidents (e.g.,
   * hitting some objects, piercing their cheek). In all these scenarios, it
   * makes most sense to go back to the MoveAbovePlate state from where users
   * can indicate preference for different bite selection, being done with eating,
   * continuing without acquisition, or waiting in the staging location.
   * Thus, the backIcon always contains the MoveAbovePlate icon image.
   * The MoveAbovePlate state itself does not have any backIcon as it automatically
   * goes to the bite selection page which has the above mentioned options.
   */
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
          </>
        )
      } else if (mealState === MEAL_STATE.R_BiteAcquisition) {
        /** If the robot is aquiring the bite and pause button is not visible,
         * only show the back button and no resume button, since the user 
         * can continue to acquiring bite again after moving above plate,
         * but if they resume this state after aquiring bite,
         * the bite selection mask will no longer work
         */
        return (
          <>
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
          </>
        )
      } else {
        /** If the robot is moving in the meal states (except the
         *  MoveAbovePlate state) of BiteAcquisition, MoveToStagingLocation,
         *  MoveToMouth, and StowingArm meal states and the pause button is not
         *  visible, then the footer shows both of the back and resume buttons
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
