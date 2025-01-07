// React Imports
import React, { useCallback, useState } from 'react'
import Button from 'react-bootstrap/Button'
import { View, Modal, TextInput, Text } from 'react-native'
import { toast } from 'react-toastify'

// Local Imports
import '../Home.css'
import { useGlobalState, MEAL_STATE } from '../../GlobalState'
import { REGULAR_CONTAINER_ID } from '../../Constants'

/**
 * The LabelGeneration component appears after the robot has moved above the plate.
 * It enables users to input labels for the food items they will be eating in the meal.
 */
const LabelGeneration = () => {
  // Get the relevant global variables
  const setMealState = useGlobalState((state) => state.setMealState)
  const setLabelGenerationConfirmed = useGlobalState((state) => state.setLabelGenerationConfirmed)
  const foodItemLabels = useGlobalState((state) => state.foodItemLabels)
  const setFoodItemLabels = useGlobalState((state) => state.setFoodItemLabels)

  // Font size for text
  let textFontSize = '3.5vh'
  // Margin
  let margin = '5vh'
  // Limit on the number of labels for food items that can be inputted
  const maxLabels = 20
  // Limit on the number of label buttons that can be displayed in a column
  // on the screen.
  const maxLabelsPerColumn = 5

  /**
   * Create a local state variable to store the current label button being edited.
   * If no label button is being edited, then set the value to null.
   */
  const [editingButton, setEditingButton] = useState(null)

  /**
   * Create a local state variable to store the text in the modal input field
   * when the user is editing a label button.
   */
  const [editButtonText, setEditButtonText] = useState('')

  /**
   * Organize the buttons for the labels in columns based on the maximum
   * labels that can be displayed in a column.
   */
  const organizeButtonColumns = () => {
    const buttonColumns = []

    Array.from(foodItemLabels).map((label, index) => {
      const columnIndex = Math.floor(index / maxLabelsPerColumn)
      if (!buttonColumns[columnIndex]) {
        buttonColumns[columnIndex] = []
      }
      buttonColumns[columnIndex].push(label)
    })

    return buttonColumns
  }

  /**
   * Render the labels inputted by the user as a list of items.
   */
  const renderLabels = () => {
    let listItems = null
    if (foodItemLabels.size > 0) {
      listItems = Array.from(foodItemLabels).map(
        (label, index) => (
          label,
          (
            <div>
              <Button
                key={index}
                variant='outline-success'
                style={{ fontSize: textFontSize, marginBottom: '2vh' }}
                onClick={() => setEditingButton({ label: label, index: index })}
              >
                {label}
              </Button>
              <Button
                variant='danger'
                style={{ fontSize: textFontSize, marginBottom: '2vh' }}
                onClick={() => {
                  setFoodItemLabels(new Set(Array.from(foodItemLabels).filter((_, i) => i !== index)))
                }}
              >
                <img style={{ width: '30px', height: 'auto' }} src='/robot_state_imgs/delete.svg' alt='delete_icon'/>
              </Button>
            </div>
          )
        )
      )
    }
    return <View style={{ flexDirection: 'column', alignItems: 'stretch' }}>{listItems}</View>
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
   * Callback function when the user clickes the "Save" button while
   * editing a label button.
   */
  const saveEditClicked = useCallback(() => {
    console.log(foodItemLabels)
    console.log(editingButton.index)
    const tempLabels = new Set(Array.from(foodItemLabels).map((label, index) => (index === editingButton.index ? editButtonText : label)))
    console.log(tempLabels)
    setFoodItemLabels(tempLabels)
    setEditingButton(null)
  })

  /**
   * Callback function when the user clicks the "Begin Meal!" button.
   */
  const beginMealClicked = useCallback(() => {
    if (foodItemLabels.size === 0) {
      // Display a toast message to the user if they have not inputted any labels
      toast.error('Please input at least one label for the food items you will be eating!', {
        containerId: REGULAR_CONTAINER_ID,
        toastId: 'emptyName'
      })
    } else {
      setLabelGenerationConfirmed(true)
      setMealState(MEAL_STATE.U_UnderstandPlate)
    }
        
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
          <p className='transitionMessage' style={{ marginBottom: margin, marginTop: '1', fontSize: textFontSize }}>
            What will you be eating today?
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
          <label style={{ fontSize: '2.5vh' }}>
            List the food items you&apos;ll be eating this meal: <input type='text' className='inputLabel' />
            <Button
              variant='warning'
              className='mx-2 btn-md'
              height='90%'
              size='md'
              onClick={onAddLabelClicked}
              style={{ fontSize: '2.5vh' }}
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
        <Modal transparent={true} visible={!!editingButton} animationType='slide' onRequestClose={() => setEditingButton(null)}>
          <View style={{ flex: 1, justifyContent: 'center', alignItems: 'center', backgroundColor: 'rgba(0,0,0,0.5)' }}>
            <View style={{ width: '80%', backgroundColor: 'white', borderRadius: 20, padding: 20, alignItems: 'center' }}>
              <Text style={{ fontSize: textFontSize, marginBottom: 15 }}>Edit Button Text</Text>
              <TextInput
                style={{
                  fontSize: textFontSize,
                  width: '100%',
                  borderBottomWidth: 5,
                  borderBottomColor: 'green',
                  padding: 20,
                  marginBottom: 15
                }}
                defaultValue={editingButton?.label}
                onChangeText={(text) => setEditButtonText(text)}
                autoFocus={true}
              />
              <View style={{ flex: 1, flexDirection: 'row', justifyContent: 'center', alignItems: 'center' }}>
                <Button
                  variant='success'
                  className='justify-content-center mx-2 mb-2'
                  height='90%'
                  size='lg'
                  onClick={saveEditClicked}
                  style={{ fontSize: textFontSize }}
                >
                  Save
                </Button>
                <Button
                  variant='danger'
                  className='justify-content-center mx-2 mb-2'
                  height='90%'
                  size='lg'
                  onClick={() => {
                    setEditingButton(null)
                  }}
                  style={{ fontSize: textFontSize }}
                >
                  Cancel
                </Button>
              </View>
            </View>
          </View>
        </Modal>
      </>
    )
  })

  // Render the component
  return <>{fullPageView()}</>
}

export default LabelGeneration
