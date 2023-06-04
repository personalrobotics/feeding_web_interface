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
import { REALSENSE_WIDTH, REALSENSE_HEIGHT, MOVING_STATE_ICON_DICT } from '../Constants'
import { useWindowSize, convertRemToPixels, scaleWidthHeightToWindow } from '../../helpers'
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
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Icons and other parameters for the footer buttons
  let pauseIcon = '/robot_state_imgs/pause_button_icon.svg'
  let backIcon = props.backMealState ? MOVING_STATE_ICON_DICT[props.backMealState] : ''
  let resumeIcon = MOVING_STATE_ICON_DICT[mealState]
  let phantomButtonIcon = '/robot_state_imgs/phantom_view_image.svg'

  // Get the size of the robot's live video stream.
  let size = useWindowSize()
  const margin = convertRemToPixels(1)
  let { width } = scaleWidthHeightToWindow(size, REALSENSE_WIDTH, REALSENSE_HEIGHT, margin, margin, margin, margin)

  // Width of Back and Resume buttons
  let backResumeButtonWidth = width / 2
  // Width of Pause button
  let pauseButtonWidth = width
  // Height of Footer buttons and icons. In landscape, the button will be shorter in height, since it is wider.
  let footerButtonHeight = isPortrait ? '95px' : '50px'
  let footerIconHeight = isPortrait ? '87px' : '42px'
  // Width of Footer icon images in buttons
  let footerIconWidth = isPortrait ? '82px' : '90px'
  // Size of Footer buttons' font
  let footerFontSize = '150%'
  // Margin around footer buttons
  let footerMargin = 3

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
                height: footerButtonHeight,
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
                      marginBottom: '0',
                      fontSize: footerFontSize,
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
                      width: footerIconWidth,
                      height: footerIconHeight
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
    [pauseIcon, footerButtonHeight, footerIconHeight, pauseButtonWidth, footerFontSize, footerIconWidth]
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
              marginRight: footerMargin,
              '--bs-btn-padding-y': '0rem'
            }}
          >
            <View style={{ flexDirection: 'row', justifyContent: 'center', margin: '1px' }}>
              <View style={{ justifyContent: 'center' }}>
                <p
                  className='transitionMessage'
                  style={{ marginBottom: '0', fontSize: footerFontSize, color: 'black', fontWeight: 'bold', padding: '0' }}
                >
                  Back
                </p>
              </View>
              <View>
                <img style={{ width: footerIconWidth, height: footerIconHeight }} src={backIcon} alt='back_icon' className='center' />
              </View>
            </View>
          </Button>
        </>
      )
    },
    [backIcon, backResumeButtonWidth, footerButtonHeight, footerMargin, footerIconHeight, footerFontSize, footerIconWidth]
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
              marginLeft: footerMargin,
              width: backResumeButtonWidth,
              height: footerButtonHeight
            }}
          >
            <View style={{ flexDirection: 'row', justifyContent: 'center', marginLeft: isPortrait ? '11px' : '' }}>
              <View style={{ justifyContent: 'center' }}>
                <p
                  className='transitionMessage'
                  style={{
                    marginBottom: '0',
                    marginLeft: '1px',
                    fontSize: footerFontSize,
                    color: 'black',
                    fontWeight: 'bold',
                    padding: '0'
                  }}
                >
                  Resume
                </p>
              </View>
              <View>
                <img style={{ width: footerIconWidth, height: footerIconHeight }} src={resumeIcon} alt='resume_icon' className='center' />
              </View>
            </View>
          </Button>
        </>
      )
    },
    [isPortrait, resumeIcon, backResumeButtonWidth, footerButtonHeight, footerMargin, footerFontSize, footerIconWidth, footerIconHeight]
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
            style={{ width: footerIconWidth, height: footerIconHeight }}
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
