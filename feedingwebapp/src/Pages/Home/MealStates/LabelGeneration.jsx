// React Imports
import React, { useCallback, useRef, useState } from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'
import { useMediaQuery } from 'react-responsive'
import { View, Modal } from 'react-native'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { useROS, createROSService, createROSServiceRequest } from '../../../ros/ros_helpers'
import { ROS_SERVICE_NAMES } from '../../Constants'

/**
 * The LabelGeneration component appears after the robot has moved above the plate.
 * It enables users to input labels for the food items they will be eating in the meal.
 */
const LabelGeneration = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  const labelGenerationConfirmed = useGlobalState((state) => state.labelGenerationConfirmed)
  const setLabelGenerationConfirmed = useGlobalState((state) => state.setLabelGenerationConfirmed)
  const foodItemLabels = useGlobalState((state) => state.foodItemLabels)
  const setFoodItemLabels = useGlobalState((state) => state.setFoodItemLabels)
  const gpt4oCaption = useGlobalState((state) => state.gpt4oCaption)
  const setGPT4oResponse = useGlobalState((state) => state.setGPT4oResponse)

  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Font size for text
  let textFontSize = '3.5vh'
  // Margin
  let margin = '5vh'
  // Indicator of how to arrange screen elements based on orientation
  let dimension = isPortrait ? 'column' : 'row'
  // Limit on the number of labels for food items that can be inputted
  const maxLabels = 10
  
  /**
   * Create a local state variable to store the current label button being edited.
   * If no label button is being edited, then set the value to null.
   */
  const [editingButton, setEditingButton] = useState(null)
  
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
  let serviceDetails = ROS_SERVICE_NAMES[MEAL_STATE.U_LabelGeneration]
  let invokeGPT4oService = useRef(createROSService(ros.current, serviceDetails.serviceName, serviceDetails.messageType))

  /** 
   * Render the labels inputted by the user
   * as a list of items
   */
  const renderLabels = () => {
    let listItems = null
    if (foodItemLabels.size > 0) {
      listItems = [...foodItemLabels].map((label, index) =>
        <li key={index} style={{fontSize: textFontSize}}>{label}</li>
      )
    }
    return <ul>{listItems}</ul>
  }

  /**
   * Callback function when the user clicks the "Add Label" button.
   */
  const onAddLabelClicked = useCallback(() => {
    const newLabel = document.querySelector('.inputLabel').value
    if (foodItemLabels.size === undefined) {
      setFoodItemLabels(new Set([newLabel]))
    } else if (foodItemLabels.size < maxLabels) {
      setFoodItemLabels(new Set([...foodItemLabels, newLabel]))
    }
  }, [foodItemLabels, setFoodItemLabels, maxLabels])

  /**
   * Callback function when the user clicks the "Begin Meal!" button.
   */
  const beginMealClicked = useCallback(() => {
    console.log('beginMealClicked')
    // Create a service request for the GPT-4o service
    const inputLabels = Array.from(foodItemLabels)
    let request = createROSServiceRequest({ input_labels: inputLabels })
    // Call the GPT-4o service
    let service = invokeGPT4oService.current
    service.callService(request, (response) => {setGPT4oResponse(response.caption)})
    console.log('GPT-4o service called' + gpt4oCaption)
    setLabelGenerationConfirmed(true)
    setMealState(MEAL_STATE.U_BiteSelection)
  }, [setLabelGenerationConfirmed, setMealState])

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
            Please label the food items you will be eating in this meal.
          </p>
        </View>
        <View 
          style={{
            flex: 1,
            flexDirection: 'column',
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%',
            height: '50%'
          }}
        >
          <label>
            List the food items you&apos;ll be eating this meal: <input type='text' className='inputLabel' />
            <Button 
              variant='warning'
              className='mx-2 btn-md'
              height='90%'
              size='md' 
              onClick={onAddLabelClicked}
              >
              Add Label
            </Button>
          </label>
        </View>
        <View
          style={{
            flex: 10,
            flexDirection: 'column',
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%',
            height: '100%'
          }}
        >
          {renderLabels()}          
        </View>
        <View
          style={{
            flex: 2,
            marginRight: '2.5%',
            flexDirection: 'row',
            justifyContent: 'flex-end',
            width: '100%',
            height: '100%'
          }}
        >
          <Button
            variant='success'
            className='mx-2 btn-huge'
            size='lg'
            style={{ fontSize: textFontSize, marginTop: '0', marginBottom: '0', height: '90%' }}
            onClick={beginMealClicked}
          >
            Begin Meal!
          </Button>
        </View>
      </>
    )
  })

  // Render the component
  return <>{fullPageView()}</>
}

export default LabelGeneration
