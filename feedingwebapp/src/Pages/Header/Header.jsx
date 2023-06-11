// React imports
import React, { useCallback, useEffect, useState } from 'react'
import { useMediaQuery } from 'react-responsive'
// The NavBar is the navigation toolbar at the top
import Navbar from 'react-bootstrap/Navbar'
import Nav from 'react-bootstrap/Nav'
// The Button is used for stop icon at the top
import Button from 'react-bootstrap/Button'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'
// Toast generates a temporary pop-up with a timeout.
import { ToastContainer, toast } from 'react-toastify'
import 'react-toastify/dist/ReactToastify.css'
// ROS imports
import { useROS } from '../../ros/ros_helpers'

// Local imports
import { ROS_CHECK_INTERVAL_MS, NON_MOVING_STATES } from '../Constants'
import { useGlobalState, APP_PAGE, MEAL_STATE } from '../GlobalState'
import LiveVideoModal from './LiveVideoModal'

/**
 * The Header component consists of the navigation bar (which has buttons Home,
 * Settings, Lock and Robot Connection Icon and Video). Live video view is toggled on and off by
 * clicking "Video", and the ToastContainer popup that specifies when the user
 * cannot click Settings.
 */
const Header = (props) => {
  // Create a local state variable to toggle on/off the video
  // TODO: Since this local state variable is in the header, the LiveVideoModal
  // continues showing even if the state changes. Is this desirable? Perhaps
  // it should close if the state changes?
  const [videoShow, setVideoShow] = useState(false)
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  // useROS gives us access to functions to configure and interact with ROS.
  let { ros } = useROS()
  const [isConnected, setIsConncected] = useState(ros.isConnected)
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Flag to check if the width matches intended devices' width
  const deviceWidthMatches = useMediaQuery({ query: '(min-width:429px)' })
  // Sizes of header elements (fontSize, width, height)
  let textFontSize = isPortrait ? '2.2vh' : '2.2vw'
  let lockIconWidth = isPortrait ? '4vh' : '4vw'
  let lockIconHeight = isPortrait ? '5vh' : '5vw'
  let lockImageHeight = isPortrait ? '4vh' : '4vw'
  let headerMargin = isPortrait ? '0.3vh' : '0.3vw'

  // Check ROS connection every ROS_CHECK_INTERVAL_MS milliseconds
  useEffect(() => {
    const interval = setInterval(() => {
      setIsConncected(ros.isConnected)
    }, ROS_CHECK_INTERVAL_MS)
    return () => clearInterval(interval)
  }, [ros, setIsConncected])

  // Get the relevant global state variables
  const mealState = useGlobalState((state) => state.mealState)
  const setAppPage = useGlobalState((state) => state.setAppPage)
  const paused = useGlobalState((state) => state.paused)
  const teleopIsMoving = useGlobalState((state) => state.teleopIsMoving)

  /**
   * When the Home button in the header is clicked, return to the Home page.
   */
  const homeClicked = useCallback(() => {
    setAppPage(APP_PAGE.Home)
  }, [setAppPage])

  /**
   * Callback function for when the user indicates that they want to move the
   * robot to locate the plate.
   */
  const locatePlateClicked = useCallback(() => {
    console.log('locatePlateClicked')
    setMealState(MEAL_STATE.U_PlateLocator)
  }, [setMealState])

  /**
   * Callback function for when the user indicates that they are done with their
   * meal.
   */
  const doneEatingClicked = useCallback(() => {
    console.log('doneEatingClicked')
    setMealState(MEAL_STATE.R_StowingArm)
  }, [setMealState])

  /**
   * When the Settings button in the header is clicked, if the meal has not yet
   * started, take the user to the settings menu. Else, ask them to complete
   * or terminate the meal because modifying settings.
   */
  const settingsClicked = useCallback(() => {
    if (mealState === MEAL_STATE.U_PreMeal || mealState === MEAL_STATE.U_PostMeal) {
      setAppPage(APP_PAGE.Settings)
    } else {
      toast('Please complete or terminate the feeding process to access Settings.')
    }
  }, [mealState, setAppPage])

  // Render the component. The NavBar will stay fixed even as we vertically scroll.
  return (
    <>
      {/**
       * The ToastContainer is an alert that pops up on the top of the screen
       * and has a timeout.
       */}
      <ToastContainer style={{ fontSize: '4vh' }} />
      {/**
       * The NavBar has five elements, Home and Settings, on the left side and three
       * elements: Lock Icon, Robot Connection Status Icon, and Video; on the right side.
       */}
      <Navbar
        collapseOnSelect
        expand='lg'
        bg='dark'
        variant='dark'
        sticky='top'
        style={{ width: '100vw', '--bs-navbar-padding-x': '0rem', '--bs-navbar-padding-y': '0rem' }}
      >
        <Navbar
          id='responsive-navbar-nav'
          bg='dark'
          variant='dark'
          style={{ '--bs-navbar-padding-x': '0rem', '--bs-navbar-padding-y': '0.2rem' }}
        >
          <Nav className='m-auto'>
            <Nav.Link
              onClick={homeClicked}
              className='text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2'
              style={{ fontSize: textFontSize }}
            >
              Home
            </Nav.Link>
            <Nav.Link
              onClick={settingsClicked}
              className='text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2'
              style={{ fontSize: textFontSize }}
            >
              Settings
            </Nav.Link>
            {deviceWidthMatches ? (
              <>
                <Nav.Link
                  onClick={locatePlateClicked}
                  className='text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2'
                  style={{ fontSize: textFontSize }}
                >
                  Locate Plate
                </Nav.Link>
                <Nav.Link
                  onClick={doneEatingClicked}
                  className='text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2'
                  style={{ fontSize: textFontSize }}
                >
                  Meal Done
                </Nav.Link>
              </>
            ) : (
              <></>
            )}
            {NON_MOVING_STATES.has(mealState) || paused || (mealState === MEAL_STATE.U_PlateLocator && teleopIsMoving === false) ? (
              <div>
                <Button
                  variant='danger'
                  disabled={true}
                  style={{
                    marginLeft: headerMargin,
                    marginRight: headerMargin,
                    width: lockIconWidth,
                    height: lockIconHeight,
                    opacity: 1,
                    '--bs-btn-padding-y': '0rem',
                    '--bs-btn-padding-x': '0rem'
                  }}
                >
                  <img
                    style={{ width: lockIconWidth, height: lockImageHeight }}
                    src='/robot_state_imgs/lock_icon_image.svg'
                    alt='lock_icon_img'
                    className='center'
                  />
                </Button>
              </div>
            ) : (
              <></>
            )}
            {isConnected ? (
              <div>
                <p className='connectedDiv' style={{ fontSize: textFontSize, margin: headerMargin }}>
                  🔌
                </p>
              </div>
            ) : (
              <div>
                <p className='notConnectedDiv' style={{ fontSize: textFontSize, marginLeft: headerMargin, marginRight: headerMargin }}>
                  ⛔
                </p>
              </div>
            )}
            <Nav.Link
              onClick={() => setVideoShow(true)}
              className='text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2'
              style={{ fontSize: textFontSize }}
            >
              Video
            </Nav.Link>
          </Nav>
        </Navbar>
      </Navbar>
      {/**
       * The LiveVideoModal toggles on and off with the Video button and shows the
       * robot's live camera feed.
       */}
      <LiveVideoModal webVideoServerURL={props.webVideoServerURL} show={videoShow} onHide={() => setVideoShow(false)} />
    </>
  )
}
Header.propTypes = {
  // The URL of the ROS web video server
  webVideoServerURL: PropTypes.string.isRequired
}

export default Header
