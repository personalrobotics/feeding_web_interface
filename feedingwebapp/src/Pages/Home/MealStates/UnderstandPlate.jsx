// React Imports
import React, { useCallback, useRef, useState, useEffect } from 'react'
import { View } from 'react-native'

// Local Imports
import '../Home.css'
import {
  ROS_ACTIONS_NAMES,
  ROS_ACTION_STATUS_CANCEL_GOAL,
  ROS_ACTION_STATUS_EXECUTE,
  ROS_ACTION_STATUS_SUCCEED,
  ROS_ACTION_STATUS_ABORT,
  ROS_ACTION_STATUS_CANCELED,
  LABEL_GENERATION_STATUS_SUCCESS
} from '../../Constants'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { useROS, createROSActionClient, callROSAction, destroyActionClient } from '../../../ros/ros_helpers'

/**
 * The UnderstandPlate component displays a circle progress bar while
 * invoking the GPT-4o service to generate a caption for the food items
 * and generating semantic label + mask pairs for each food item.
 */
const UnderstandPlate = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  const gpt4oCaption = useGlobalState((state) => state.gpt4oCaption)
  const foodItemLabels = useGlobalState((state) => state.foodItemLabels)
  const setGPT4oCaption = useGlobalState((state) => state.setGPT4oCaption)

  let textFontSize = '3.5vh'

  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  const [actionStatus, setActionStatus] = useState({
    actionStatus: null
  })
  /**
   * Create the ROS Action to invoke GPT-4o to transform labels into a caption
   * for VLM detection. This is created in useRef to avoid re-creating the client
   * upon re-renders.
   */
  let actionDetails = ROS_ACTIONS_NAMES[MEAL_STATE.U_UnderstandPlate]
  let generateCaptionAction = useRef(createROSActionClient(ros.current, actionDetails.actionName, actionDetails.messageType))

  const responseCallback = useCallback(
    (response) => {
      if (response.response_type === 'result' && response.values.status === LABEL_GENERATION_STATUS_SUCCESS) {
        console.log('GPT-4o Response:', response)
        setActionStatus({
          actionStatus: ROS_ACTION_STATUS_SUCCEED
        })
        setGPT4oCaption(response.values.caption)
        setMealState(MEAL_STATE.U_BiteSelection)
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
    [setGPT4oCaption, setMealState]
  )

  const feedbackCallback = useCallback(
    (feedbackMsg) => {
      console.log('GPT-4o Feedback:', feedbackMsg)
      setActionStatus({
        actionStatus: ROS_ACTION_STATUS_EXECUTE,
        feedback: feedbackMsg.values.feedback
      })
    },
    [setActionStatus]
  )

  /**
   * Invoke the GenerateCaption action upon this component being mounted and
   * activate the progress bar.
   */
  useEffect(() => {
    // Call the GPT-4o action to generate a caption for the food items
    console.log('reached use effect')
    let action = generateCaptionAction.current
    console.log('action:', action)

    const inputLabels = Array.from(foodItemLabels)
    console.log('input labels:', inputLabels)
    callROSAction(action, { input_labels: inputLabels }, feedbackCallback, responseCallback)

    setActionStatus({
      actionStatus: ROS_ACTION_STATUS_EXECUTE
    })

    return () => {
      // Destroy the action client
      destroyActionClient(action)
    }
  }, [generateCaptionAction, foodItemLabels, responseCallback, feedbackCallback])

  const actionStatusText = useCallback(() => {
    switch (actionStatus.actionStatus) {
      case ROS_ACTION_STATUS_EXECUTE:
        if (actionStatus.feedback) {
          let time = actionStatus.feedback.elapsed_time.sec + actionStatus.feedback.elapsed_time.nanosec / 10 ** 9
          return 'Robot is labeling the food items...' + (Math.round(time * 100) / 100).toString() + ' sec'
        } else {
          return 'Robot is labeling the food items...'
        }
      case ROS_ACTION_STATUS_SUCCEED:
        if (gpt4oCaption.length > 0) {
          return 'Robot has labeled the food items'
        } else {
          return 'Retry labeling the food items'
        }
      case ROS_ACTION_STATUS_ABORT:
        return 'Error in labeling task'
      case ROS_ACTION_STATUS_CANCELED:
        return 'Labeling task was canceled'
      default:
        return ''
    }
  }, [actionStatus, gpt4oCaption])

  return (
    <>
      <View
        style={{
          flex: 1,
          alignItems: 'center',
          justifyContent: 'center',
          width: '100%',
          height: '100%'
        }}
      >
        <h1 style={{ textAlign: 'center', fontSize: textFontSize }}>{actionStatusText()}</h1>
      </View>
    </>
  )
}

export default UnderstandPlate
