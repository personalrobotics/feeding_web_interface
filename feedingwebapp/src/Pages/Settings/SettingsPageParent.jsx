// React imports
import React from 'react'
import { useMediaQuery } from 'react-responsive'
import Button from 'react-bootstrap/Button'
// The Modal is a screen that appears on top of the main app, and can be toggled
// on and off.
import Modal from 'react-bootstrap/Modal'
import PropTypes from 'prop-types'
import { View } from 'react-native'

// Local imports
import { useGlobalState } from '../GlobalState'

/**
 * The SettingsPageParent component handles visual layout that is common across
 * all settings pages.
 */
const SettingsPageParent = (props) => {
  // Get relevant global state variables
  const settingsPresets = useGlobalState((state) => state.settingsPresets)

  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Rendering variables
  let textFontSize = isPortrait ? 3.0 : 5.0
  let sizeSuffix = 'vh'

  return (
    <>
      <View
        style={{
          flex: 1,
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          width: '100%',
          height: '100%'
        }}
      >
        <View
          style={{
            flex: 4,
            flexDirection: 'row',
            alignItems: 'center',
            justifyContent: 'center',
            width: '100%',
            height: '100%'
          }}
        >
          <p style={{ textAlign: 'center', fontSize: textFontSize.toString() + sizeSuffix, margin: 0 }} className='txt-huge'>
            {props.title}
          </p>
          <Button variant='secondary' disabled size='lg' style={{ fontSize: textFontSize.toString() + sizeSuffix, marginLeft: '1rem' }}>
            {settingsPresets.current}
          </Button>
        </View>
        <View
          style={{
            flex: 32,
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%'
          }}
        >
          {props.children}
        </View>
        <View
          style={{
            flex: 4,
            flexDirection: 'column',
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%'
          }}
        >
          <Button
            variant='success'
            className='mx-2 mb-2 btn-huge'
            size='lg'
            style={{
              fontSize: textFontSize.toString() + sizeSuffix,
              width: '90%',
              height: '90%',
              color: 'black',
              padding: '0rem'
            }}
            onClick={props.doneCallback}
          >
            Done
          </Button>
        </View>
      </View>
      <Modal
        show={props.modalShow}
        onHide={props.modalOnHide}
        size='lg'
        aria-labelledby='contained-modal-title-vcenter'
        backdrop='static'
        keyboard={false}
        centered
        id='robotMotionModal'
        fullscreen={false}
        dialogClassName='modal-90w'
        style={{
          '--bs-modal-padding': '0rem'
        }}
      >
        <Modal.Header closeButton />
        <Modal.Body style={{ overflow: 'hidden' }}>
          <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', height: '65vh' }}>{props.modalChildren}</View>
        </Modal.Body>
      </Modal>
    </>
  )
}
SettingsPageParent.propTypes = {
  // The title of the page
  title: PropTypes.string.isRequired,
  // The content to include in the page
  children: PropTypes.node.isRequired,
  // The callback for when the done button is clicked
  doneCallback: PropTypes.func.isRequired,
  // Parameters for the modal
  modalShow: PropTypes.bool,
  modalOnHide: PropTypes.func,
  modalChildren: PropTypes.node
}
SettingsPageParent.defaultProps = {
  modalShow: false,
  modalOnHide: () => {},
  modalChildren: <></>
}

export default SettingsPageParent
