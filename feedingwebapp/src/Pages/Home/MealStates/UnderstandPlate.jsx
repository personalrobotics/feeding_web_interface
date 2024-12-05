// React Imports
import React, { useCallback, useRef, useState, useEffect } from 'react'
import { View } from 'react-native'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { useROS, createROSActionClient, callROSAction, destroyActionClient } from '../../../ros/ros_helpers'
import { ROS_ACTIONS_NAMES } from '../../Constants'
import CircleProgressBar from './CircleProgressBar'


/**
 * The DetectingFoods component displays a circle progress bar while
 * invoking the GPT-4o service to generate a caption for the food items
 * and generating semantic label + mask pairs for each food item.
 */
const DetectingFoods = (props) => {
  // Get the relevant global variables  
  const setMealState = useGlobalState((state) => state.setMealState)
  const gpt4oCaption = useGlobalState((state) => state.gpt4oCaption)
  const foodItemLabels = useGlobalState((state) => state.foodItemLabels)
  const setGPT4oCaption = useGlobalState((state) => state.setGPT4oCaption)
  // Font size for text
  let textFontSize = '3.5vh'
  // Margin
  let margin = '5vh'

  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  const [progress, setProgress] = useState(0)

  const [isRequestOngoing, setIsRequestOngoing] = useState(false);

  /**
   * Create the ROS Action to invoke GPT-4o to transform labels into a caption
   * for VLM detection. This is created in useRef to avoid re-creating the client
   * upon re-renders.
   */
  let actionDetails = ROS_ACTIONS_NAMES[MEAL_STATE.U_UnderstandPlate]
  let generateCaptionAction = useRef(createROSActionClient(ros.current, actionDetails.actionName, actionDetails.messageType))

  const responseCallback = useCallback((response) => {
    console.log('GPT-4o Response:', response)
    setGPT4oCaption(response)
    setMealState(MEAL_STATE.U_BiteSelection)
  }, [setGPT4oCaption, setMealState])

  const feedbackCallback = useCallback((feedbackMsg) => {
    console.log('GPT-4o Feedback:', feedbackMsg)
    if (feedbackMsg.values.feedback) {
      setProgress(feedbackMsg.values.feedback)
    }
  })

  /**
   * Activate the circle progress bar when the ROS Action to invoke GPT-4o is called.
   */
  useEffect(() => {
    // Call the GPT-4o action to generate a caption for the food items
    console.log('reached use effect')
    let action = generateCaptionAction.current

    const inputLabels = Array.from(foodItemLabels)
    console.log('input labels:', inputLabels)
    callROSAction(
      action, 
      { input_labels: inputLabels }, 
      feedbackCallback, 
      responseCallback
    )
    
    return () => { 
      // Destroy the action client
      destroyActionClient(action)
    }
  }, [generateCaptionAction, foodItemLabels, responseCallback, feedbackCallback])

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
          <CircleProgressBar proportion={progress} />
        </View>
      </>
    )
  })

  return <>{fullPageView()}</>
}

export default DetectingFoods
