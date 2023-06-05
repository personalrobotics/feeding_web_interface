// React Imports
import React, { useCallback, useEffect, useRef, useState } from 'react'
import Button from 'react-bootstrap/Button'
import { useMediaQuery } from 'react-responsive'
import { View } from 'react-native'
// PropTypes is used to validate that the used props are in fact passed to this
// Component
import PropTypes from 'prop-types'
import Row from 'react-bootstrap/Row'
import Col from 'react-bootstrap/Col'

// Local Imports
import '../Home.css'
import { useROS, createROSActionClient, callROSAction, destroyActionClient } from '../../../ros/ros_helpers'
import { useWindowSize, convertRemToPixels, scaleWidthHeightToWindow } from '../../../helpers'
import MaskButton from '../../../buttons/MaskButton'
import {
  REALSENSE_WIDTH,
  REALSENSE_HEIGHT,
  ROS_ACTIONS_NAMES,
  CAMERA_FEED_TOPIC,
  ROS_ACTION_STATUS_CANCEL_GOAL,
  ROS_ACTION_STATUS_EXECUTE,
  ROS_ACTION_STATUS_SUCCEED,
  ROS_ACTION_STATUS_ABORT,
  ROS_ACTION_STATUS_CANCELED,
  SEGMENTATION_STATUS_SUCCESS
} from '../../Constants'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { MOVING_STATE_ICON_DICT } from '../../Constants'

/**
 * The BiteSelection component appears after the robot has moved above the plate,
 * plate. It enables users to select their desired food item.
 *
 * @param {boolean} debug - whether to run it in debug mode (e.g., if you aren't
 *        simulatenously running the robot) or not
 */
const BiteSelection = (props) => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  const setDesiredFoodItem = useGlobalState((state) => state.setDesiredFoodItem)
  // Get icon image for move to mouth
  let moveToMouthImage = MOVING_STATE_ICON_DICT[MEAL_STATE.R_MovingToMouth]
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Width and Height of skip acquisition button
  let skipAcquisitionButtonWidth = isPortrait ? '300px' : '160px'
  let skipAcquisitionButtonHeight = isPortrait ? '200px' : '106px'
  // Factor to modify video size in landscape which has less space than portrait
  let landscapeSizeFactor = 0.68

  /**
   * Create a local state variable to store the detected masks, the
   * status of the action, and how many times the user has clicked the
   * image.
   */
  const [actionResult, setActionResult] = useState(null)
  const [numImageClicks, setNumImageClicks] = useState(0)
  /**
   * NOTE: We slightly abuse the ROS_ACTION_STATUS values in this local state
   * variable, by using it as a proxy for whether the robot is executing, has
   * succeeded, has been canceled, or has had an error. This is to avoid
   * creating an extra enum.
   */
  const [actionStatus, setActionStatus] = useState({
    actionStatus: null
  })

  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  /**
   * Create the ROS Action Client. This is created in useRef to avoid
   * re-creating it upon re-renders.
   */
  let { actionName, messageType } = ROS_ACTIONS_NAMES[MEAL_STATE.U_BiteSelection]
  let segmentFromPointAction = useRef(createROSActionClient(ros.current, actionName, messageType))

  /**
   * Callback function for when the user indicates that they want to move the
   * robot to locate the plate.
   */
  const locatePlateClicked = useCallback(() => {
    console.log('locatePlateClicked')
    setMealState(MEAL_STATE.U_PlateLocator)
  }, [setMealState])

  /**
   * Callback function for when the user wants to move to mouth position.
   */
  const moveToMouth = useCallback(() => {
    setMealState(MEAL_STATE.R_MovingToMouth)
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
   * Callback function for when the user clicks the button for a food item.
   */
  const foodItemClicked = useCallback(
    (event) => {
      let food_i = Number(event.target.value)
      setDesiredFoodItem(actionResult.detected_items[food_i])
      setMealState(MEAL_STATE.R_BiteAcquisition)
    },
    [actionResult, setDesiredFoodItem, setMealState]
  )

  /**
   * Callback function for when the action sends feedback. It updates the
   * actionStatus local state variable.
   *
   * @param {object} feedbackMsg - the feedback message sent by the action
   */
  const feedbackCallback = useCallback(
    (feedbackMsg) => {
      setActionStatus({
        actionStatus: ROS_ACTION_STATUS_EXECUTE,
        feedback: feedbackMsg.values.feedback
      })
    },
    [setActionStatus]
  )

  /**
   * Callback function for when the action sends a response. It updates the
   * actionStatus local state variable and stores the returned masks if the
   * action succeeded.
   *
   * TODO: What should happen if the action succeeds, but a response is never
   * received e.g., due to a momentary networking issue? We should likely allow
   * the user to press a button to move on if a response hasn't been received
   * within n seconds of making the action call. Or allow the user to retry the
   * action call (maybe they would refresh the page anyway, which might be ok).
   */
  const responseCallback = useCallback(
    (response) => {
      if (response.response_type === 'result' && response.values.status === SEGMENTATION_STATUS_SUCCESS) {
        setActionStatus({
          actionStatus: ROS_ACTION_STATUS_SUCCEED
        })
        setActionResult(response.values)
      } else {
        if (
          response.response_type === 'cancel' ||
          response.values === ROS_ACTION_STATUS_CANCEL_GOAL ||
          response.values === ROS_ACTION_STATUS_CANCELED
        ) {
          setActionStatus({
            actionStatus: ROS_ACTION_STATUS_CANCELED
          })
        } else {
          setActionStatus({
            actionStatus: ROS_ACTION_STATUS_ABORT
          })
        }
      }
    },
    [setActionStatus, setActionResult]
  )

  // Get the size of the robot's live video stream.
  let size = useWindowSize()
  const margin = convertRemToPixels(1)
  const { width, height, scaleFactor } = scaleWidthHeightToWindow(size, REALSENSE_WIDTH, REALSENSE_HEIGHT, margin, margin, margin, margin)
  const imgSrc = `${props.webVideoServerURL}/stream?topic=${CAMERA_FEED_TOPIC}&width=${Math.round(width)}&height=${Math.round(
    height
  )}&quality=20`

  /**
   * Callback function for when the user clicks the image of the plate.
   *
   * TODO: As we integrate this with SegmentAnything, we should think more
   * carefully about how to handle the case where the user clicks on a pixel,
   * changes their mind, and then clicks on another pixel. Ideally, we'd cancel
   * the old goal and only use the new goal. But first of all, that requires
   * canceling one (not all) active goals, which is not currently supported by
   * roslibjs ROS2 actions. We could implement this functionality on the action
   * server side, but that depends on whether calls to SegmentAnything are
   * blocking or not. If we do stick with the current functionality where users
   * clicks are disregarded until the previous goal is completed, then we should
   * make that clear to the user.
   */
  const imageClicked = useCallback(
    (event) => {
      // Get the position of the click relative to the raw RealSense image.
      let rect = event.target.getBoundingClientRect()
      let x = event.clientX - rect.left // x position within the image.
      let y = event.clientY - rect.top // y position within the image.
      let x_raw = Math.round(x / scaleFactor) // x position within the raw image.
      let y_raw = Math.round(y / scaleFactor) // y position within the raw image.
      console.log('Left? : ' + x_raw + ' ; Top? : ' + y_raw + '.')

      // Call the food segmentation ROS action
      callROSAction(
        segmentFromPointAction.current,
        { seed_point: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'image_frame' }, point: { x: x_raw, y: y_raw, z: 0.0 } } },
        /**
         * Only register callbacks the first time to avoid multiple callbacks for
         * the same action call.
         */
        numImageClicks === 0 ? feedbackCallback : null,
        numImageClicks === 0 ? responseCallback : null
      )
      setNumImageClicks(numImageClicks + 1)
    },
    [segmentFromPointAction, scaleFactor, numImageClicks, feedbackCallback, responseCallback]
  )

  /**
   * Cancel any running actions when the component unmounts
   */
  useEffect(() => {
    let action = segmentFromPointAction.current
    return () => {
      destroyActionClient(action)
    }
  }, [segmentFromPointAction])

  /**
   * Render the appropriate text and/or buttons based on the action status.
   *
   * @returns {JSX.Element} the action status text to render
   */
  const actionStatusText = useCallback(() => {
    switch (actionStatus.actionStatus) {
      case ROS_ACTION_STATUS_EXECUTE:
        if (actionStatus.feedback) {
          let elapsed_time = actionStatus.feedback.elapsed_time.sec + actionStatus.feedback.elapsed_time.nanosec / 10 ** 9
          return (
            <React.Fragment>
              <h5 style={{ textAlign: 'center' }}>Detecting food... ({Math.round(elapsed_time * 100) / 100} sec)</h5>
              <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
              <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
              <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
            </React.Fragment>
          )
        } else {
          return (
            <React.Fragment>
              <h5 style={{ textAlign: 'center' }}>Detecting food... </h5>
              <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
              <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
              <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
            </React.Fragment>
          )
        }
      case ROS_ACTION_STATUS_SUCCEED:
        if (actionResult && actionResult.detected_items && actionResult.detected_items.length > 0) {
          // Get the parameters to display the mask as buttons
          let imgSize = { width: width, height: height }
          let maskScaleFactor = scaleFactor

          let [maxWidth, maxHeight] = [0, 0]
          for (let detected_item of actionResult.detected_items) {
            if (detected_item.roi.width > maxWidth) {
              maxWidth = detected_item.roi.width
            }
            if (detected_item.roi.height > maxHeight) {
              maxHeight = detected_item.roi.height
            }
          }
          let buttonSize = { width: maxWidth * maskScaleFactor, height: maxHeight * maskScaleFactor }

          return (
            <>
              <h5 style={{ textAlign: 'center' }}>Select a food, or retry by clicking the image.</h5>
              <Row>
                {actionResult.detected_items.map((detected_item, i) => (
                  <Col key={i} className='justify-content-center' style={{ padding: '0' }}>
                    <MaskButton
                      buttonSize={buttonSize}
                      imgSrc={imgSrc}
                      imgSize={imgSize}
                      maskSrc={'data:image/jpeg;base64,' + detected_item.mask.data}
                      invertMask={true}
                      maskScaleFactor={maskScaleFactor}
                      maskBoundingBox={detected_item.roi}
                      onClick={foodItemClicked}
                      value={i.toString()}
                    />
                  </Col>
                ))}
              </Row>
            </>
          )
        } else {
          return <h4 style={{ textAlign: 'center' }}>Food detection succeeded</h4>
        }
      case ROS_ACTION_STATUS_ABORT:
        /**
         * TODO: Just displaying that the robot faced an error is not useful
         * to the user. We should think more carefully about what different
         * error cases might arise, and change the UI accordingly to instruct
         * users on how to troubleshoot/fix it.
         */
        return <h3 style={{ textAlign: 'center' }}>Error in food detection</h3>
      case ROS_ACTION_STATUS_CANCELED:
        return <h3 style={{ textAlign: 'center' }}>Food detection canceled</h3>
      default:
        return (
          <React.Fragment>
            <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
            <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
            <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
            <h3 style={{ textAlign: 'center' }}>&nbsp;</h3>
          </React.Fragment>
        )
    }
  }, [actionStatus, actionResult, width, height, scaleFactor, foodItemClicked, imgSrc])

  /**
   * Get the robot's live video stream.
   *
   * @param {number} currentWidth the adjusted width in pixels
   * @param {number} currentHeight the adjusted height in pixels
   *
   * @returns {JSX.Element} the robot's live video stream
   */
  let showVideo = function (currentWidth, currentHeight) {
    return (
      <React.Fragment>
        <h5 style={{ textAlign: 'center' }}>Click on image to select food.</h5>
        <img
          src={imgSrc}
          alt='Live video feed from the robot'
          style={{ width: currentWidth, height: currentHeight, display: 'block' }}
          onClick={imageClicked}
        />
      </React.Fragment>
    )
  }

  // Get the button for continue without acquiring bite
  let withoutAcquireButton = function () {
    return (
      <div style={{ display: 'block', width: '100%' }} className='outer'>
        {/* Ask the user whether they want to continue without acquisition by moving to above plate position */}
        <h5 style={{ textAlign: 'center' }}>Skip acquisition.</h5>
        {/* Icon to move to mouth */}
        <Row className='justify-content-center'>
          <Button
            variant='warning'
            className='mx-2 mb-2 btn-huge'
            size='lg'
            onClick={moveToMouth}
            style={{ width: skipAcquisitionButtonWidth, height: skipAcquisitionButtonHeight }}
          >
            <img
              src={moveToMouthImage}
              style={{ width: isPortrait ? '250px' : '140px', height: isPortrait ? '150px' : '86px' }}
              alt='move_to_mouth_image'
              className='center'
            />
          </Button>
        </Row>
      </div>
    )
  }

  // Get the continue button when debug mode is enabled.
  let debugOptions = function () {
    /* If the user is running in debug mode, give them the option to skip */
    props.debug ? (
      <Button
        variant='secondary'
        className='justify-content-center mx-2 mb-2'
        size='lg'
        onClick={() => setMealState(MEAL_STATE.R_BiteAcquisition)}
      >
        Continue (Debug Mode)
      </Button>
    ) : (
      <></>
    )
  }

  // Render the component
  return (
    <>
      {/**
       * In addition to selecting their desired food item, the user has two
       * other options on this page:
       *   - If their desired food item is not visible on the plate, they can
       *     decide to teleoperate the robot until it is visible.
       *   - Instead of selecting their next bite, the user can indicate that
       *     they are done eating.
       */}
      <div style={{ display: 'block', textAlign: 'center' }}>
        <Button className='doneButton' style={{ fontSize: '18px', marginTop: '3px' }} onClick={locatePlateClicked}>
          🍽️ Locate Plate
        </Button>
        <Button className='doneButton' style={{ fontSize: '18px', marginTop: '3px' }} onClick={doneEatingClicked}>
          ✅ Done Eating
        </Button>
      </div>
      {isPortrait ? (
        <React.Fragment>
          <center>
            {showVideo(width, height)}
            {/* Display the action status and/or results */}
            {actionStatusText()}
          </center>
          {withoutAcquireButton()}
          {debugOptions()}
        </React.Fragment>
      ) : (
        <View style={{ flexDirection: 'row' }}>
          <View style={{ paddingHorizontal: 40 }}>{showVideo(width * landscapeSizeFactor, height * landscapeSizeFactor)}</View>
          <View style={{ justifyContent: 'center', width: '450px' }}>
            {actionStatusText()}
            {withoutAcquireButton()}
            {debugOptions}
          </View>
        </View>
      )}
    </>
  )
}
BiteSelection.propTypes = {
  /**
   * Whether to run it in debug mode (e.g., if you aren't simulatenously running
   * the robot) or not
   */
  debug: PropTypes.bool.isRequired,
  // The URL of the web video server
  webVideoServerURL: PropTypes.string.isRequired
}

export default BiteSelection
