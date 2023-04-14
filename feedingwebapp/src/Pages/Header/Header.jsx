// React imports
import React from 'react'
// The NavBar is the navigation toolbar at the top
import Navbar from 'react-bootstrap/Navbar'
import Nav from 'react-bootstrap/Nav'
// Toast generates a temporary pop-up with a timeout.
import { ToastContainer, toast } from 'react-toastify'
import 'react-toastify/dist/ReactToastify.css'

// Local imports
import { useGlobalState, APP_PAGE, MEAL_STATE } from '../GlobalState'
import LiveVideoModal from './LiveVideoModal'

/**
 * The Header component consists of the navigation bar (which has buttons Home,
 * Settings, and Video), the live video view that is toggled on and off by
 * clicking "Video", and the ToastContainer popup that specifies when the user
 * cannot click Settings.
 */
const Header = () => {
  // Create a local state variable to toggle on/off the video
  const [videoShow, setVideoShow] = React.useState(false)

  // Get the relevant global state variables
  const mealState = useGlobalState((state) => state.mealState)
  const setAppPage = useGlobalState((state) => state.setAppPage)

  /**
   * When the Home button in the header is clicked, return to the Home page.
   */
  function homeClicked() {
    setAppPage(APP_PAGE.Home)
  }

  /**
   * When the Settings button in the header is clicked, if the meal has not yet
   * started, take the user to the settings menu. Else, ask them to complete
   * or terminate the meal because modifying settings.
   */
  function settingsClicked() {
    if (mealState == MEAL_STATE.U_PreMeal || mealState == MEAL_STATE.U_PostMeal) {
      setAppPage(APP_PAGE.Settings)
    } else {
      toast('Please complete or terminate the feeding process to access Settings.')
    }
  }

  // Render the component
  // TODO: Find a way to make the NavBar fixed even as we vertically scroll the
  // contents of the Home/Settings page.
  return (
    <>
      {/**
       * The ToastContainer is an alert that pops up on the top of the screen
       * and has a timeout.
       */}
      <ToastContainer style={{ fontSize: '28px' }} />
      {/**
       * The NavBar has two elements, Home and Settings, on the left side and one
       * element, Video, on the right side.
       */}
      <Navbar collapseOnSelect expand='lg' bg='dark' variant='dark'>
        <Navbar id='responsive-navbar-nav'>
          <Nav className='me-auto'>
            <Nav.Link
              onClick={homeClicked}
              className='text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2'
              style={{ fontSize: '175%' }}
            >
              Home
            </Nav.Link>
            <Nav.Link
              onClick={settingsClicked}
              className='text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2'
              style={{ fontSize: '175%' }}
            >
              Settings
            </Nav.Link>
          </Nav>
          <Nav>
            <Nav.Link
              onClick={() => setVideoShow(true)}
              className='text-dark bg-info border border-info rounded mx-1 btn-lg btn-huge p-2'
              style={{ fontSize: '175%' }}
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
      <LiveVideoModal show={videoShow} onHide={() => setVideoShow(false)} />
    </>
  )
}

export default Header
