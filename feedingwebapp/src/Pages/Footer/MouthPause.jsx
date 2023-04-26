// React imports
import React, { useRef } from 'react'
import Button from 'react-bootstrap/Button'
// The Modal is a screen that appears on top of the main app, and can be toggled
// on and off.
import Modal from 'react-bootstrap/Modal'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'

import { useGlobalState, MEAL_STATE } from '../GlobalState'

/**
 * The PauseModal gives the user the option to resume the robot when they are
 * ready.
 */
function PauseModal(props) {
  const ref = useRef(null)
  const setMealState = useGlobalState((state) => state.setMealState)

  /**
   * Callback function for if the user decides to cancel the bite.
   *
   * TODO: Think more carefully about what cancelBite at this stage should do!
   * Maybe replace with a more descriptive button (e.g., "return to staging.")
   */
  function priorState() {
    console.log('prior state')
    setMealState(MEAL_STATE.U_BiteInitiation)
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
            ◀️ Return to Bite Initiation
          </Button>
          <Button
            className='bg-warning rounded btn-hugeE'
            style={{ fontSize: '35px', marginLeft: '8%', fontWeight: 'bold', marginBottom: '8%' }}
            size='lg'
            onClick={props.resumeClicked}
          >
            ▶️ Proceed to Bite Done
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
