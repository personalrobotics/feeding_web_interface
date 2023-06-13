// import circle progress bar from progress bar package
import { Circle } from './progressbar.js'
// React imports
import React, { useEffect, useState } from 'react'
import { useMediaQuery } from 'react-responsive'
// PropTypes is used to validate that the used props are in fact passed to this Component
import PropTypes from 'prop-types'

/**
 * A functional component to render full circle progress bars
 * in robot moving screens. It is imported in RobotMotion.jsx.
 *
 * @param {number} proportion - the current progress level of robot's motion
 */
export default function CircleProgressBar(props) {
  // define a local state variable to keep track of progress bar creation
  const [bar, setBar] = useState(null)
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // define sizes of progressbar (width, height, fontsize) in portrait and landscape
  let textFontSize = isPortrait ? '9vh' : '5vw'
  let circleSize = isPortrait ? '40vh' : '16vw'

  // useEffect React Hook is used to synchronize with RobotMotion.jsx data.
  useEffect(() => {
    // Effect: rendering circle progress bar
    // if circle progress bar does not exist, create it; else render it with text
    if (bar === null) {
      setBar(
        new Circle('#container', {
          // green color indicating smooth progression
          color: '#008000',
          // width of the stroke to be applied to the shape
          strokeWidth: 4,
          // width of the trail to be applied to the shape before stroke fills in
          trailWidth: 1,
          // specifies text style
          text: {
            style: {
              fontFamily: '"Raleway", Helvetica, sans-serif',
              fontSize: textFontSize,
              position: 'absolute',
              left: '50%',
              top: '50%',
              padding: '0px',
              margin: '0px',
              transform: 'translate(-50%, -50%)'
            }
          }
        })
      )
    } else {
      // sets progress instantly without animation and clears all animations for path.
      bar.set(props.proportion)
      // sets text to given a string.
      bar.setText(Math.round(props.proportion * 100) + '%')
    }
    // everytime items in dependency array (the second argument) update, useEffect runs.
  }, [setBar, bar, props.proportion, textFontSize])
  // render HTML
  return <div id='container' style={{ margin: '20px', width: circleSize, height: circleSize, position: 'relative' }}></div>
}

// progress proportion corresponding with the motion the robot is executing
CircleProgressBar.propTypes = { proportion: PropTypes.number.isRequired }
