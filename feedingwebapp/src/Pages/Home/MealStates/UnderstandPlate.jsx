// React Imports
import React, { useCallback } from 'react'

// Local Imports
import '../Home.css'
import { useROS, createROSService } from '../../../ros/ros_helpers'
import { ROS_ACTIONS_NAMES } from '../../Constants'

/**
 * The DetectingFoods component displays a circle progress bar while
 * invoking the GPT-4o service to generate a caption for the food items
 * and generating semantic label + mask pairs for each food item.
 */
const DetectingFoods = (props) => {
  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  /**
   * Create the ROS Service to invoke GPT-4o to transform labels into a caption
   * for VLM detection. This is created in useRef to avoid re-creating the client
   * upon re-renders.
   */
  let actionDetails = ROS_ACTIONS_NAMES[MEAL_STATE.U_UnderstandPlate]
  let generateCaptionAction = useRef(createROSActionClient(ros.current, actionDetails.actionName, actionDetails.messageType))

  /** Get the full page view
   *
   * @returns {JSX.Element} the full page view
   */
  const fullPageView = useCallback(() => {
    return (
      <>
        <View
          style={{
            flex: 2,
            flexDirection: 'column',
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%',
            height: '100%'
          }}
        >
          <p className='transitionMessage' style={{ marginBottom: margin, marginTop: '0', fontSize: textFontSize }}>
            Understanding the plate...
          </p>
          <CircleProgressBar value={progress} text={`${progress}%`} />
        </View>
      </>
    )
  })
}
