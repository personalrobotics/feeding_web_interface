// React imports
import React, { useRef } from 'react'
import Button from 'react-bootstrap/Button'
// The Modal is a screen that appears on top of the main app, and can be toggled
// on and off.
import Modal from 'react-bootstrap/Modal'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'
// local imports
import { move_above_plate_dict, bite_acquistion_dict, move_to_staging_dict, move_to_mouth_dict, stow_arm_dict } from '../Constants'
import { MEAL_STATE, useGlobalState } from '../GlobalState'

/**
 * The PauseModal gives the user the option to resume the robot when they are
 * ready.
 */
function PauseModal(props) {
  const ref = useRef(null)
  var meal_state = useGlobalState((state) => state.mealState)
  console.log(meal_state)
  //var current_meal_state_text = feeding_dict[meal_state]
  var setMealState = useGlobalState((state) => state.setMealState)
  console.log(setMealState)
  var prev_text = ''
  var next_text = ''
  var text1 = '◀️ Return to '
  var text2 = '▶️ Proeed to '
  var prev_state = null
  if (meal_state == MEAL_STATE.R_MovingAbovePlate) {
    prev_text = move_above_plate_dict.prev_text
    next_text = move_above_plate_dict.next_text
    prev_state = move_above_plate_dict.prev_state
  } else if (meal_state == MEAL_STATE.R_BiteAcquisition) {
    prev_text = bite_acquistion_dict.prev_text
    next_text = bite_acquistion_dict.next_text
    prev_state = bite_acquistion_dict.prev_state
  } else if (meal_state == MEAL_STATE.R_MovingToStagingLocation) {
    prev_text = move_to_staging_dict.prev_text
    next_text = move_to_staging_dict.next_text
    prev_state = move_to_staging_dict.prev_state
  } else if (meal_state == MEAL_STATE.R_MovingToMouth) {
    prev_text = move_to_mouth_dict.prev_text
    next_text = move_to_mouth_dict.next_text
    prev_state = move_to_mouth_dict.prev_state
  } else {
    // meal_state == MEAL_STATE.R_Stowing_Arm
    prev_text = stow_arm_dict.prev_text
    next_text = stow_arm_dict.next_text
    prev_state = stow_arm_dict.prev_state
  }
  text1 = text1 + prev_text
  text2 = text2 + next_text
  console.log(text1)
  console.log(text2)
  console.log(prev_state)

  /**
   * Callback function for if the user decides to cancel the bite.
   *
   * TODO: Think more carefully about what cancelBite at this stage should do!
   * Maybe replace with a more descriptive button (e.g., "return to staging.")
   */
  function priorState() {
    console.log('prior state')
    setMealState(prev_state)
  }
  return (
    <>
      {/**
       * TODO:
       *   - Center the resume button in the modal so it works for all screen widths.
       *   - Determine why this Modal doesn't show an x-mark whereas LiveVideoModal
       *     does (probably has something to do with modal content being too big?)
       */}
      <Modal
        show={props.show}
        onHide={props.resumeClicked}
        size='lg'
        aria-labelledby='contained-modal-title-vcenter'
        backdrop='static'
        keyboard={false}
        centered
        id='pauseModal'
        fullscreen={true}
        ref={ref}
      >
        <Modal.Header>
          <Modal.Title id='contained-modal-title-vcenter'>⏸️ Paused!</Modal.Title>
        </Modal.Header>
        <Modal.Body style={{ paddingLeft: '10px', overflow: 'hidden' }}>
          <p className='transitionMessage' style={{ marginBottom: '10px', marginTop: '0px', fontSize: '24px' }}>
            Resume the feeding session when you are ready.
          </p>
          <Button
            className='bg-warning rounded btn-hugeE'
            style={{ fontSize: '35px', marginLeft: '8%', fontWeight: 'bold', marginBottom: '8%' }}
            size='lg'
            onClick={priorState}
          >
            {text1}
          </Button>
          <Button
            className='bg-warning rounded btn-hugeE'
            style={{ fontSize: '35px', marginLeft: '8%', fontWeight: 'bold', marginBottom: '8%' }}
            size='lg'
            onClick={props.resumeClicked}
          >
            {text2}
          </Button>
        </Modal.Body>
      </Modal>
    </>
  )
}
PauseModal.propTypes = {
  show: PropTypes.bool,
  resumeClicked: PropTypes.func
}

export default PauseModal
