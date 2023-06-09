// React imports
import React, { useCallback } from 'react'
import { MDBFooter } from 'mdb-react-ui-kit'
import Button from 'react-bootstrap/Button'
import { View } from 'react-native'
import Row from 'react-bootstrap/Row'
import { useMediaQuery } from 'react-responsive'
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
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  let phantomButtonIcon = '/robot_state_imgs/phantom_view_image.svg'
  // sizes for footer buttons and icons (width, height, fontsize)
  let pauseButtonWidth = '98vw'
  let backResumeButtonWidth = '49vw'
  let footerButtonHeight = isPortrait ? '10vh' : '10vw'
  let pauseIconWidth = isPortrait ? '15vh' : '15vw'
  let backResumeIconWidth = isPortrait ? '10vh' : '10vw'
  let pauseIconHeight = isPortrait ? '10vh' : '10vw'
  let backResumeIconHeight = isPortrait ? '9vh' : '9vw'
  let pauseFontSize = isPortrait ? '7vh' : '7vw'
  let backResumeFontSize = isPortrait ? '3vh' : '3vw'
  // Margin around footer buttons
  let footerMargin = isPortrait ? '0.3vh' : '0.3vw'

  /**
   * Get the pause text and button to render in footer.
   *
   * @returns {JSX.Element} the pause text and button
   */
  const renderPauseButton = useCallback(
    (callback) => {
      return (
        <>
          <Row className='justify-content-center'>
            {/* Icon to pause */}
            <Button
              variant='danger'
              onClick={callback}
              style={{
                width: pauseButtonWidth,
                justifyContent: 'center',
                alignItems: 'center',
                height: footerButtonHeight,
                margin: footerMargin,
                '--bs-btn-padding-y': '0rem'
              }}
            >
              <View
                style={{
                  flexDirection: 'row',
                  justifyContent: 'center'
                }}
              >
                <View style={{ justifyContent: 'center' }}>
                  <p
                    className='transitionMessage'
                    style={{
                      fontSize: pauseFontSize,
                      color: 'black',
                      fontWeight: 'bold',
                      padding: '0'
                    }}
                  >
                    Pause
                  </p>
                </View>
                <View>
                  <img
                    style={{
                      width: pauseIconWidth,
                      height: pauseIconHeight
                    }}
                    src={pauseIcon}
                    alt='pause_icon'
                    className='center'
                  />
                </View>
              </View>
            </Button>
          </Row>
        </>
      )
    },
    [pauseIcon, footerButtonHeight, pauseButtonWidth, pauseIconHeight, pauseIconWidth, pauseFontSize, footerMargin]
  )

  /**
   * Get the back text and button to render in footer.
   *
   * @returns {JSX.Element} the back text and button
   */
  const renderBackButton = useCallback(
    (callback) => {
      return (
        <>
          <Button
            variant='warning'
            onClick={callback}
            style={{
              width: backResumeButtonWidth,
              height: footerButtonHeight,
              margin: footerMargin,
              '--bs-btn-padding-y': '0rem'
            }}
          >
            <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
              <View style={{ justifyContent: 'center' }}>
                <p
                  className='transitionMessage'
                  style={{ marginBottom: '0', fontSize: backResumeFontSize, color: 'black', fontWeight: 'bold', padding: '0' }}
                >
                  Back
                </p>
              </View>
              <View>
                <img
                  style={{ width: backResumeIconWidth, height: backResumeIconHeight }}
                  src={backIcon}
                  alt='back_icon'
                  className='center'
                />
              </View>
            </View>
          </Button>
        </>
      )
    },
    [backIcon, footerMargin, backResumeButtonWidth, backResumeFontSize, footerButtonHeight, backResumeIconHeight, backResumeIconWidth]
  )

  /**
   * Get the resume text and button to render in footer.
   *
   * @returns {JSX.Element} the resume text and button
   */
  const renderResumeButton = useCallback(
    (callback) => {
      return (
        <>
          <Button
            variant='success'
            onClick={callback}
            style={{
              margin: footerMargin,
              width: backResumeButtonWidth,
              height: footerButtonHeight,
              '--bs-btn-padding-y': '0rem'
            }}
          >
            <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
              <View style={{ justifyContent: 'center' }}>
                <p
                  className='transitionMessage'
                  style={{
                    marginBottom: '0',
                    fontSize: backResumeFontSize,
                    color: 'black',
                    fontWeight: 'bold'
                  }}
                >
                  Resume
                </p>
              </View>
              <View>
                <img
                  style={{ width: backResumeIconWidth, height: backResumeIconHeight }}
                  src={resumeIcon}
                  alt='resume_icon'
                  className='center'
                />
              </View>
            </View>
          </Button>
        </>
      )
    },
    [resumeIcon, footerMargin, backResumeButtonWidth, backResumeFontSize, footerButtonHeight, backResumeIconHeight, backResumeIconWidth]
  )

  /**
   * Get the phantom view to render in footer. This is used as a placeholder
   * when the back button or resume button are disabled.
   *
   * @returns {JSX.Element} the phantom view
   */
  let renderPhantomButton = function () {
    return (
      <>
        <Button
          variant='ghost'
          disabled={true}
          style={{
            backgroundColor: 'transparent',
            border: 'none',
            marginLeft: footerMargin,
            marginRight: footerMargin,
            width: backResumeButtonWidth,
            height: footerButtonHeight,
            '--bs-btn-padding-y': '0rem'
          }}
        >
          <img
            style={{ width: backResumeIconWidth, height: backResumeIconHeight }}
            src={phantomButtonIcon}
            alt='phantom_button_img'
            className='center'
          />
        </Button>
      </>
    )
  }

  // Render the component
  return (
    <>
      <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
        <div className='text-center' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)', paddingBottom: '5px', paddingTop: '5px' }}>
          {props.paused ? (
            <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
              <View>{props.backCallback ? renderBackButton(props.backCallback) : renderPhantomButton()}</View>
              <View>{props.resumeCallback ? renderResumeButton(props.resumeCallback) : renderPhantomButton()}</View>
            </View>
          ) : (
            renderPauseButton(props.pauseCallback)
          )}
        </div>
      </MDBFooter>
    </>
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
