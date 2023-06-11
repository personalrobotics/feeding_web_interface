// React imports
import React from 'react'
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
  let backResumeButtonWidth = '47vw'
  let footerHeight = isPortrait ? '11vh' : '11vw'
  let pauseFontSize = isPortrait ? '7vh' : '7vw'
  let backResumeFontSize = '6vw'
  // Margins around footer buttons
  let footerLeftRightMargin = isPortrait ? '1.5vh' : '1.5vw'
  let footerTopBottomMargin = isPortrait ? '0.3vh' : '0.3vw'
  // Text list for footer buttons
  const texts = ['Pause', 'Back', 'Resume']
  // Icon list for footer buttons
  const icons = [pauseIcon, backIcon, resumeIcon]
  // Variant list for footer buttons
  const variants = ['danger', 'warning', 'success']
  // font sizes for footer button texts
  const fontSizes = [pauseFontSize, backResumeFontSize, backResumeFontSize]
  // button widths for footer
  const buttonWidths = [pauseButtonWidth, backResumeButtonWidth, backResumeButtonWidth]
  // callback functions for footer button click
  const callbacks = [props.pauseCallback, props.backCallback, props.resumeCallback]

  /**
   * Get the footer text and button to render in footer.
   *
   * @returns {JSX.Element} the footer text and button
   */
  function renderFooterButton(index) {
    if (index === 3) {
      return (
        <>
          <Button
            variant='ghost'
            disabled={true}
            style={{
              backgroundColor: 'transparent',
              border: 'none',
              marginTop: footerTopBottomMargin,
              marginLeft: footerLeftRightMargin,
              marginRight: footerLeftRightMargin,
              marginBottom: footerTopBottomMargin,
              width: backResumeButtonWidth,
              height: footerHeight,
              '--bs-btn-padding-y': '0rem'
            }}
          >
            <img style={{ width: '100%', height: footerHeight }} src={phantomButtonIcon} alt='phantom_button_img' className='center' />
          </Button>
        </>
      )
    } else {
      return (
        <>
          <Row className='justify-content-center'>
            {/* Icon to pause */}
            <Button
              variant={variants[index]}
              onClick={callbacks[index]}
              style={{
                width: buttonWidths[index],
                justifyContent: 'center',
                alignItems: 'center',
                height: footerHeight,
                marginTop: footerTopBottomMargin,
                marginLeft: footerLeftRightMargin,
                marginRight: footerLeftRightMargin,
                marginBottom: footerTopBottomMargin,
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
                      fontSize: fontSizes[index],
                      color: 'black',
                      fontWeight: 'bold',
                      padding: '0',
                      textAlign: 'right'
                    }}
                  >
                    {texts[index]}
                  </p>
                </View>
                <View style={{ flex: 1, justifyContent: 'center', alignItems: 'flex-start' }}>
                  <div style={{ textAlign: 'left' }}>
                    <img
                      style={{
                        width: '100%',
                        height: footerHeight
                      }}
                      src={icons[index]}
                      alt='icon_img'
                    />
                  </div>
                </View>
              </View>
            </Button>
          </Row>
        </>
      )
    }
  }

  // Render the component
  return (
    <>
      <MDBFooter bgColor='dark' className='text-center text-lg-left fixed-bottom'>
        <div className='text-center' style={{ backgroundColor: 'rgba(0, 0, 0, 0.2)', paddingBottom: '5px', paddingTop: '5px' }}>
          {props.paused ? (
            <View style={{ flexDirection: 'row', justifyContent: 'center' }}>
              <View>{props.backCallback ? renderFooterButton(1) : renderFooterButton(3)}</View>
              <View>{props.resumeCallback ? renderFooterButton(2) : renderFooterButton(3)}</View>
            </View>
          ) : (
            renderFooterButton(0)
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
