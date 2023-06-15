// React imports
import React, { useCallback } from 'react'
import { MDBFooter } from 'mdb-react-ui-kit'
import Button from 'react-bootstrap/Button'
import { View } from 'react-native'
import { useMediaQuery } from 'react-responsive'
import Row from 'react-bootstrap/Row'
// PropTypes is used to validate that the used props are in fact passed to this Component
import PropTypes from 'prop-types'
// Local imports
import { MOVING_STATE_ICON_DICT } from '../Constants'
import { useGlobalState } from '../GlobalState'

/**
 * The Footer shows a pause button. When users click it, the app tells the robot
 * to immediately pause and displays a back button that allows them to return to
 * previous state and a resume button that allows them to resume current state.
 *
 * @param {bool} paused - whether the robot is currently paused
 * @param {function} pauseCallback - callback function for when the pause button
 *     is clicked
 * @param {function} backCallback - callback function for when the back button
 *     is clicked. If null, don't render the back button.
 * @param {string} backMealState - the meal state to return to when the back
 *     button is clicked. If null, don't render the back button.
 * @param {function} resumeCallback - callback function for when the resume
 *     button is clicked. If null, don't render the resume button.
 */
const Footer = (props) => {
  // Get the current meal state
  const mealState = useGlobalState((state) => state.mealState)
  // Icons for the footer buttons
  let pauseIcon = '/robot_state_imgs/pause_button_icon.svg'
  let backIcon = props.backMealState ? MOVING_STATE_ICON_DICT[props.backMealState] : ''
  let resumeIcon = MOVING_STATE_ICON_DICT[mealState]
  let phantomIcon = '/robot_state_imgs/phantom_view_image.svg'
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Margins around footer buttons
  let footerLeftRightMargin = isPortrait ? '1.6vh' : '1.6vw'
  let footerTopBottomMargin = isPortrait ? '0.3vh' : '0.3vw'
  // A single nested object with all footer buttons' properties
  const buttonConfig = {
    pause: {
      text: 'Pause',
      icon: pauseIcon,
      disabled: false,
      variant: 'danger',
      callback: props.pauseCallback,
      buttonWidth: '98vw',
      buttonHeight: isPortrait ? '11vh' : '11vw',
      fontSize: isPortrait ? '7vh' : '7vw',
      backgroundColor: null
    },
    back: {
      text: 'Back',
      icon: backIcon,
      disabled: false,
      variant: 'warning',
      callback: props.backCallback,
      buttonWidth: '47vw',
      buttonHeight: isPortrait ? '11vh' : '11vw',
      fontSize: '6vw',
      backgroundColor: null
    },
    resume: {
      text: 'Resume',
      icon: resumeIcon,
      disabled: false,
      variant: 'success',
      callback: props.resumeCallback,
      buttonWidth: '47vw',
      buttonHeight: isPortrait ? '11vh' : '11vw',
      fontSize: '6vw',
      backgroundColor: null
    },
    phantom: {
      text: null,
      icon: phantomIcon,
      disabled: true,
      variant: 'ghost',
      callback: null,
      buttonWidth: '47vw',
      buttonHeight: isPortrait ? '11vh' : '11vw',
      fontSize: '6vw',
      backgroundColor: 'transparent'
    }
  }

  /**
   * Get the footer text and button to render in footer.
   *
   * @param {object} config - a single nested object with properties of footer buttons
   *
   * @returns {JSX.Element} the footer text and button
   */
  const renderFooterButton = useCallback(
    (config) => {
      return (
        <>
          <Row className='justify-content-center'>
            <Button
              variant={config.variant}
              disabled={config.disabled}
              onClick={config.callback}
              style={{
                backgroundColor: config.backgroundColor,
                border: 'none',
                marginTop: footerTopBottomMargin,
                marginLeft: footerLeftRightMargin,
                marginRight: footerLeftRightMargin,
                marginBottom: footerTopBottomMargin,
                width: config.buttonWidth,
                height: config.buttonHeight,
                justifyContent: 'center',
                alignItems: 'center',
                '--bs-btn-padding-y': '0rem',
                '--bs-btn-padding-x': '0rem'
              }}
            >
              <View
                style={{
                  flexDirection: 'row',
                  justifyContent: 'center'
                }}
              >
                <View style={{ flex: 1, justifyContent: 'center' }}>
                  <p
                    className='transitionMessage'
                    style={{
                      marginBottom: '0',
                      fontSize: config.fontSize,
                      color: 'black',
                      fontWeight: 'bold',
                      padding: '0',
                      textAlign: 'right'
                    }}
                  >
                    {config.text}
                  </p>
                </View>
                <View style={{ flex: 1, justifyContent: 'center', alignItems: 'flex-start' }}>
                  <div style={{ textAlign: 'left' }}>
                    <img
                      style={{
                        width: '100%',
                        height: config.buttonHeight
                      }}
                      src={config.icon}
                      alt='icon_img'
                    />
                  </div>
                </View>
              </View>
            </Button>
          </Row>
        </>
      )
    },
    [footerLeftRightMargin, footerTopBottomMargin]
  )

  // Render the component
  return (
    <View>
      <MDBFooter bgColor='dark' className='text-center text-lg-left' style={{ width: '100vw' }}>
        <div className='text-center' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)', paddingBottom: '5px', paddingTop: '5px' }}>
          {props.paused ? (
            <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
              <View>{props.backCallback ? renderFooterButton(buttonConfig.back) : renderFooterButton(buttonConfig.phantom)}</View>
              <View>{props.resumeCallback ? renderFooterButton(buttonConfig.resume) : renderFooterButton(buttonConfig.phantom)}</View>
            </View>
          ) : (
            renderFooterButton(buttonConfig.pause)
          )}
        </div>
      </MDBFooter>
    </View>
  )
}
Footer.propTypes = {
  paused: PropTypes.bool.isRequired,
  pauseCallback: PropTypes.func.isRequired,
  // If any of the below three are null, the Footer won't render that button
  resumeCallback: PropTypes.func,
  backCallback: PropTypes.func,
  backMealState: PropTypes.string
}

export default Footer
