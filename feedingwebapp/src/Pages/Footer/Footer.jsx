// React imports
import React, { useState } from 'react'
import { MDBFooter } from 'mdb-react-ui-kit'
import Button from 'react-bootstrap/Button'
import { View } from 'react-native'
import Row from 'react-bootstrap/Row'

// Local imports
import { robot_moving_state_icon_image_dict } from '../Constants'
import { useGlobalState } from '../GlobalState'

/**
 * The Footer shows a pause button. When users click it, the app tells the robot
 * to immediately pause and displays a model that gives them the option to resume.
 */
const Footer = () => {
  // Set the current meal state
  const setMealState = useGlobalState((state) => state.setMealState)
  // Get the current meal state
  const mealState = useGlobalState((state) => state.mealState)
  console.log(mealState)
  // Get the previous meal state
  const previousMealState = useGlobalState((state) => state.previousMealState)
  console.log(previousMealState)
  // Create a local variable for storing previous state icon image
  var previous_state_image = robot_moving_state_icon_image_dict[previousMealState]
  // Create a local variable for storing current state icon image
  var current_state_image = robot_moving_state_icon_image_dict[mealState]
  // Local state variable to track of visibility of pause button
  const [pause_button_visible, setPauseButtonVisible] = useState(true)

  /**
   * When the back button is clicked, go back to previous state.
   */
  function backButtonClicked() {
    // set meal state to previous state
    setMealState(previousMealState)
    // we call setPauseButtonVisible with a new value. React will re-render the Footer component.
    setPauseButtonVisible(true)
  }

  return (
    <>
      {/**
       * The footer has a pause button. A resume button and a back button are shown when the pause button is clicked.
       */}
      <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
        <div className='text-center p-3' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)' }}>
          {pause_button_visible ? (
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
                  alt='pause_button_image'
                  className='center'
                />
              </Button>
            </Row>
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
                  <img
                    style={{ width: '450px', height: '270px' }}
                    src={previous_state_image}
                    alt='previous_state_icon_img'
                    className='center'
                  />
                </Button>
              </View>
              <View>
                <p className='transitionMessage' style={{ marginBottom: '0', fontSize: '170%', color: 'white', fontWeight: 'bold' }}>
                  ▶️ Resume
                </p>
                {/* Icon to resume */}
                <Button
                  variant='success'
                  onClick={() => setPauseButtonVisible(true)}
                  style={{ marginLeft: 10, marginRight: 10, width: '150px', height: '100px' }}
                >
                  <img
                    style={{ width: '450px', height: '270px' }}
                    src={current_state_image}
                    alt='current_state_icon_img'
                    className='center'
                  />
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
