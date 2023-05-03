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
import { pause_modal_state_info_dict } from '../Constants'
import { useGlobalState } from '../GlobalState'

/**
 * The PauseModal gives the user the option to resume the robot when they are
 * ready.
 */
function PauseModal(props) {
  const ref = useRef(null)
  // check which meal state we are currently at
  var meal_state = useGlobalState((state) => state.mealState)
  // function to set a meal state
  var setMealState = useGlobalState((state) => state.setMealState)
  var return_text = '◀️ Return by '
  var proceed_text = '▶️ Continue with '
  // access current meal state info from dictionary in constants.js
  var state_info_list = pause_modal_state_info_dict[meal_state]
  var prev_text_from_dict = state_info_list[0]
  var next_text_from_dict = state_info_list[1]
  var prev_state_from_dict = state_info_list[2]
  return_text = return_text + prev_text_from_dict
  proceed_text = proceed_text + next_text_from_dict

  /**
   * Function to return to previous meal state from the current
   * meal state, if user selects "Return to....." option in the pause modal.
   */
  function returnButtonCallback() {
    setMealState(prev_state_from_dict)
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
            onClick={returnButtonCallback}
          >
            {return_text}
          </Button>
          <Button
            className='bg-warning rounded btn-hugeE'
            style={{ fontSize: '35px', marginLeft: '8%', fontWeight: 'bold', marginBottom: '8%' }}
            size='lg'
            onClick={props.resumeClicked}
          >
            {proceed_text}
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
