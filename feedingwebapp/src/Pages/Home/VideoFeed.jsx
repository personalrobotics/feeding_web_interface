// React Imports
import React, { useCallback, useEffect, useState } from 'react'
// PropTypes is used to validate that the used props are in fact passed to this Component
import PropTypes from 'prop-types'

// Local Imports
import { CAMERA_FEED_TOPIC, REALSENSE_WIDTH, REALSENSE_HEIGHT } from '../Constants'
import { useWindowSize } from '../../helpers'

/**
 * Takes in an imageWidth and imageHeight, and returns a width and height that
 * maintains the same aspect ratio but fits within the window.
 *
 * @param {number} parentWidth the width of the parent DOM element in pixels
 * @param {number} parentHeight the height of the parent DOM element in pixels
 * @param {number} imageWidth the original image's width in pixels
 * @param {number} imageHeight the original image's height in pixels
 * @param {number} marginTop the desired top margin between window and image, in pixels
 * @param {number} marginBottom the desired bottom margin between window and image, in pixels
 * @param {number} marginLeft the desired left margin between window and image, in pixels
 * @param {number} marginRight the desired right margin between window and image, in pixels
 *
 * @returns {object} the width and height of the image that fits within the window and has the requested margins
 */
function scaleWidthHeightToWindow(
  parentWidth,
  parentHeight,
  imageWidth,
  imageHeight,
  marginTop = 0,
  marginBottom = 0,
  marginLeft = 0,
  marginRight = 0
) {
  // Calculate the aspect ratio of the image
  let imageAspectRatio = imageWidth / imageHeight
  // Get the aspect ratio of the available subset of the window
  let availableWidth = parentWidth - marginLeft - marginRight
  let availableHeight = parentHeight - marginTop - marginBottom
  let availableAspectRatio = availableWidth / availableHeight

  // Calculate the width and height of the image that fits within the window
  let returnWidth, returnHeight
  if (availableAspectRatio > imageAspectRatio) {
    returnHeight = Math.round(availableHeight)
    returnWidth = Math.round(imageAspectRatio * returnHeight)
  } else {
    returnWidth = Math.round(availableWidth)
    returnHeight = Math.round(returnWidth / imageAspectRatio)
  }

  // Calculate the scale factor
  let scaleFactorWidth = returnWidth / imageWidth
  let scaleFactorHeight = returnHeight / imageHeight
  let scaleFactor = (scaleFactorWidth + scaleFactorHeight) / 2

  return {
    width: returnWidth,
    height: returnHeight,
    scaleFactor: scaleFactor
  }
}

/**
 * The VideoFeed component takes in a reference to the parent DOM element, and
 * displays a video feed that takes up the maximum size within the parent DOM,
 * while maintaining the aspect ratio of the video feed and respecting specified
 * margins.
 *
 * Note that for this to work, the parent DOM element must have a specified
 * height and width that does not change with its contents. One way to achieve
 * this is to ensure the `width` and `height` props of the parent are set.
 */
const VideoFeed = (props) => {
  // Local state variables to keep track of the width and height of the video feed
  const [imgWidth, setImgWidth] = useState(0)
  const [imgHeight, setImgHeight] = useState(0)
  const [scaleFactor, setScaleFactor] = useState(0.0)

  // Callback to resize the image based on the parent width and height
  const resizeImage = useCallback(() => {
    if (!props.parent.current) {
      return
    }
    // Get the width and height of the parent DOM element
    let parentWidth = props.parent.current.clientWidth
    let parentHeight = props.parent.current.clientHeight

    // Calculate the width and height of the video feed
    let {
      width: childWidth,
      height: childHeight,
      scaleFactor: childScaleFactor
    } = scaleWidthHeightToWindow(
      parentWidth,
      parentHeight,
      REALSENSE_WIDTH,
      REALSENSE_HEIGHT,
      props.marginTop,
      props.marginBottom,
      props.marginLeft,
      props.marginRight
    )

    // Set the width and height of the video feed
    setImgWidth(childWidth)
    setImgHeight(childHeight)
    setScaleFactor(childScaleFactor)
  }, [props.parent, props.marginTop, props.marginBottom, props.marginLeft, props.marginRight])

  // When the component is first mounted, resize the image
  useEffect(() => {
    resizeImage()
  }, [resizeImage])

  /** When the resize event is triggered, the elements have not yet been laid out,
   * and hence the parent width/height might not be accurate yet based on the
   * specified flex layout. Hence, we wait until the next event cycle to resize
   * the video feed.
   */
  const resizeImageNextEventCycle = useCallback(() => {
    setTimeout(resizeImage, 0)
  }, [resizeImage])
  useWindowSize(resizeImageNextEventCycle)

  // The callback for when the image is clicked.
  const imageClicked = useCallback(
    (event) => {
      // Get the position of the click relative to the raw RealSense image.
      let rect = event.target.getBoundingClientRect()
      let x = event.clientX - rect.left // x position within the image.
      let y = event.clientY - rect.top // y position within the image.
      let x_raw = Math.round(x / scaleFactor) // x position within the raw image.
      let y_raw = Math.round(y / scaleFactor) // y position within the raw image.
      console.log('Button click on unscaled image: (' + x_raw + ', ' + y_raw + ')')

      // Call the callback function if it exists
      if (props.pointClicked) {
        props.pointClicked(x_raw, y_raw)
      }
    },
    [props, scaleFactor]
  )

  // Render the component
  return (
    <img
      src={`${props.webVideoServerURL}/stream?topic=${CAMERA_FEED_TOPIC}&width=${imgWidth}&height=${imgHeight}&quality=20`}
      alt='Live video feed from the robot'
      style={{
        width: imgWidth,
        height: imgHeight,
        display: 'block',
        alignItems: 'center',
        justifyContent: 'center'
      }}
      onClick={props.pointClicked ? imageClicked : null}
    />
  )
}
VideoFeed.propTypes = {
  // The ref to the parent DOM element. Null if the component is not yet mounted
  parent: PropTypes.object.isRequired,
  // The URL of the video feed
  webVideoServerURL: PropTypes.string.isRequired,
  // The margins around the video feed
  marginTop: PropTypes.number.isRequired,
  marginBottom: PropTypes.number.isRequired,
  marginLeft: PropTypes.number.isRequired,
  marginRight: PropTypes.number.isRequired,
  /**
   * An optional callback function for when the user clicks on the video feed.
   * This function should take in two parameters, `x` and `y`, which are the
   * coordinates of the click in the **unscaled** image (e.g., the image of
   * size REALSENSE_WIDTH x REALSENSE_HEIGHT).
   */
  pointClicked: PropTypes.func
}

export default VideoFeed