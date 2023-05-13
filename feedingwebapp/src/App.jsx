// React imports
import './App.css'
import 'bootstrap/dist/css/bootstrap.min.css'
import React from 'react'
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom'
import { RosConnection } from 'rosreact'

// Local imports
import { useGlobalState, APP_PAGE } from './Pages/GlobalState'
import Header from './Pages/Header/Header'
import Home from './Pages/Home/Home'
import Settings from './Pages/Settings/Settings'
import TestROS from './ros/TestROS'

/**
 * Determines what screen to render based on the app page specified in global
 * state.
 *
 * @param {APP_PAGE} appPage - The current app page. Must be one of the
 *        states specified in APP_PAGE.
 * @param {bool} debug - Whether to run it in debug mode or not.
 */
function getComponentByAppPage(appPage, debug) {
  switch (appPage) {
    case APP_PAGE.Home:
      // Must wrap a component in ROS tags for it to be able to connect to ROS
      return (
        <RosConnection url={process.env.REACT_APP_ROSBRIDGE_SERVER_URL} autoConnect>
          <Header />
          <Home debug={debug} />
        </RosConnection>
      )
    case APP_PAGE.Settings:
      return (
        <RosConnection url={process.env.REACT_APP_ROSBRIDGE_SERVER_URL} autoConnect>
          <Header />
          <Settings />
        </RosConnection>
      )
    default:
      return <div>Invalid app page</div>
  }
}

/**
 * Top-level configuration for the feeding web app. Renders the header and
 * main screen.
 */
function App() {
  // Get the app page
  const appPage = useGlobalState((state) => state.appPage)

  // Render the component
  return (
    <>
      <Router>
        <Routes>
          <Route exact path='/' element={getComponentByAppPage(appPage, process.env.REACT_APP_DEBUG === 'true')} />
          <Route
            exact
            path='/test_ros'
            element={
              <RosConnection url={process.env.REACT_APP_ROSBRIDGE_SERVER_URL} autoConnect>
                <TestROS />
              </RosConnection>
            }
          />
        </Routes>
      </Router>
    </>
  )
}

export default App
