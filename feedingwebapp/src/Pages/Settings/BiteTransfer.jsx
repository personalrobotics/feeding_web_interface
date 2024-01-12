// React imports
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useId, Label, SpinButton } from '@fluentui/react-components'
import Button from 'react-bootstrap/Button'
// The Modal is a screen that appears on top of the main app, and can be toggled
// on and off.
import Modal from 'react-bootstrap/Modal'
import { View } from 'react-native'

// Local imports
import { useROS, createROSService, createROSServiceRequest, getParameterValue } from '../../ros/ros_helpers'
import {
  GET_PARAMETERS_SERVICE_NAME,
  GET_PARAMETERS_SERVICE_TYPE,
  SET_PARAMETERS_SERVICE_NAME,
  SET_PARAMETERS_SERVICE_TYPE
} from '../Constants'
import { useGlobalState, MEAL_STATE, SETTINGS_STATE } from '../GlobalState'
import RobotMotion from '../Home/MealStates/RobotMotion'
import DetectingFaceSubcomponent from '../Home/MealStates/DetectingFaceSubcomponent'

/**
 * The BiteTransfer component allows users to configure parameters related to the
 * bite transfer.
 */
const BiteTransfer = () => {
  // Get relevant global state variables
  const setSettingsState = useGlobalState((state) => state.setSettingsState)
  const globalMealState = useGlobalState((state) => state.mealState)
  const setPaused = useGlobalState((state) => state.setPaused)

  // Create relevant local state variables
  // Store the current distance to mouth
  const [currentDistanceToMouth, setCurrentDistanceToMouth] = useState(null)
  const [localMealState, setLocalMealState] = useState(
    globalMealState === MEAL_STATE.U_BiteDone || globalMealState === MEAL_STATE.R_DetectingFace
      ? MEAL_STATE.R_MovingFromMouthToStagingConfiguration
      : MEAL_STATE.R_MovingToStagingConfiguration
  )
  const [waitingText, setWaitingText] = useState('Waiting to move in front of you...')
  const actionInput = useMemo(() => ({}), [])

  // Get min and max distance to mouth
  const minDistanceToMouth = 1 // cm
  const maxDistanceToMouth = 10 // cm

  // Store the props for the RobotMotion call. The first call has the robot move
  // to the staging configuration.
  const robotMotionProps = useMemo(() => {
    console.log('useMemo called with', localMealState)
    if (localMealState !== null) {
      // Start in a moving state, not a paused state
      setPaused(false)
    }
    return {
      mealState: localMealState,
      setMealState: setLocalMealState,
      nextMealState: null,
      backMealState: null,
      actionInput: actionInput,
      waitingText: waitingText
    }
  }, [localMealState, setLocalMealState, setPaused, actionInput, waitingText])

  // Rendering variables
  let textFontSize = '3.5vh'

  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  /**
   * Create the ROS Service Clients to get/set parameters.
   */
  let getParametersService = useRef(createROSService(ros.current, GET_PARAMETERS_SERVICE_NAME, GET_PARAMETERS_SERVICE_TYPE))
  let setParametersService = useRef(createROSService(ros.current, SET_PARAMETERS_SERVICE_NAME, SET_PARAMETERS_SERVICE_TYPE))

  // The first time the page is rendered, get the current distance to mouth
  useEffect(() => {
    let service = getParametersService.current
    // First, attempt to get the current distance to mouth
    let currentRequest = createROSServiceRequest({
      names: ['current.MoveToMouth.tree_kwargs.plan_distance_from_mouth']
    })
    service.callService(currentRequest, (response) => {
      console.log('Got current plan_distance_from_mouth response', response)
      if (response.values[0].type === 0) {
        // Parameter not set
        // Second, attempt to get the default distance to mouth
        let defaultRequest = createROSServiceRequest({
          names: ['default.MoveToMouth.tree_kwargs.plan_distance_from_mouth']
        })
        service.callService(defaultRequest, (response) => {
          console.log('Got default plan_distance_from_mouth response', response)
          if (response.values.length > 0) {
            setCurrentDistanceToMouth(getParameterValue(response.values[0]))
          }
        })
      } else {
        setCurrentDistanceToMouth(getParameterValue(response.values[0]))
      }
    })
  }, [getParametersService, setCurrentDistanceToMouth])

  // Callback to set the distance to mouth parameter
  const setDistanceToMouth = useCallback(
    (fullDistanceToMouth) => {
      let service = setParametersService.current
      let request = createROSServiceRequest({
        parameters: [
          {
            name: 'current.MoveToMouth.tree_kwargs.plan_distance_from_mouth',
            value: {
              type: 8, // double array
              double_array_value: fullDistanceToMouth
            }
          }
        ]
      })
      service.callService(request, (response) => {
        console.log('Got response', response)
        if (response != null && response.results.length > 0 && response.results[0].successful) {
          setCurrentDistanceToMouth(fullDistanceToMouth)
        }
      })
    },
    [setParametersService, setCurrentDistanceToMouth]
  )

  // Callback to restore the distance to mouth to the default
  const restoreToDefaultButtonClicked = useCallback(() => {
    let service = getParametersService.current
    // Attempt to get the default distance to mouth
    let defaultRequest = createROSServiceRequest({
      names: ['default.MoveToMouth.tree_kwargs.plan_distance_from_mouth']
    })
    service.callService(defaultRequest, (response) => {
      console.log('Got default plan_distance_from_mouth response', response)
      if (response.values.length > 0) {
        setDistanceToMouth(getParameterValue(response.values[0]))
      }
    })
  }, [getParametersService, setDistanceToMouth])

  // Callback to move the robot to the mouth
  const moveToMouthButtonClicked = useCallback(() => {
    setLocalMealState(MEAL_STATE.R_DetectingFace)
  }, [setLocalMealState])

  // Callback to move the robot away from the mouth
  const moveAwayFromMouthButtonClicked = useCallback(() => {
    setLocalMealState(MEAL_STATE.R_MovingFromMouthToStagingConfiguration)
    setWaitingText('Waiting to move away from you...')
  }, [setLocalMealState, setWaitingText])

  // Callback to return to the main settings page
  const doneButtonClicked = useCallback(() => {
    setSettingsState(SETTINGS_STATE.MAIN)
  }, [setSettingsState])

  // Callback for when the user changes the distance to mouth
  const onDistanceToMouthChange = useCallback(
    (_ev, data) => {
      let value = data.value ? data.value : parseFloat(data.displayValue)
      if (value < minDistanceToMouth) {
        value = minDistanceToMouth
      }
      if (value > maxDistanceToMouth) {
        value = maxDistanceToMouth
      }
      let fullDistanceToMouth = [value / 100.0, currentDistanceToMouth[1], currentDistanceToMouth[2]]
      setDistanceToMouth(fullDistanceToMouth)
    },
    [setDistanceToMouth, currentDistanceToMouth, minDistanceToMouth, maxDistanceToMouth]
  )

  // Callback to render the main contents of the page
  const distanceToMouthId = useId()
  const renderBiteTransferSettings = useCallback(() => {
    if (currentDistanceToMouth === null) {
      return (
        <>
          <View
            style={{
              flex: 1,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%'
            }}
          >
            <h5 style={{ textAlign: 'center', fontSize: textFontSize }}>Loading...</h5>
          </View>
        </>
      )
    } else {
      return (
        <>
          <View
            style={{
              flex: 8,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%'
            }}
          >
            <Label
              htmlFor={distanceToMouthId}
              style={{
                fontSize: textFontSize,
                width: '90%',
                color: 'black',
                textAlign: 'center'
              }}
            >
              Distance To Mouth (cm)
            </Label>
            <SpinButton
              value={currentDistanceToMouth[0] * 100}
              id={distanceToMouthId}
              step={0.5}
              onChange={onDistanceToMouthChange}
              appearance='filled-lighter'
              style={{
                fontSize: textFontSize,
                width: '90%',
                color: 'black'
              }}
              incrementButton={{
                'aria-label': 'Increase value by 0.5',
                'aria-roledescription': 'spinner',
                size: 'large'
              }}
            />
            <Button
              variant='warning'
              className='mx-2 mb-2 btn-huge'
              size='lg'
              style={{
                fontSize: textFontSize,
                width: '60%',
                color: 'black'
              }}
              onClick={restoreToDefaultButtonClicked}
            >
              Set to Default
            </Button>
          </View>
          <View
            style={{
              flex: 2,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%'
            }}
          >
            <Button
              variant='warning'
              className='mx-2 mb-2 btn-huge'
              size='lg'
              style={{
                fontSize: textFontSize,
                width: '90%',
                height: '90%',
                color: 'black'
              }}
              onClick={moveToMouthButtonClicked}
            >
              Move To Mouth
            </Button>
          </View>
          <View
            style={{
              flex: 2,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%'
            }}
          />
          <View
            style={{
              flex: 2,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%'
            }}
          >
            <Button
              variant='warning'
              className='mx-2 mb-2 btn-huge'
              size='lg'
              style={{
                fontSize: textFontSize,
                width: '90%',
                height: '90%',
                color: 'black'
              }}
              onClick={moveAwayFromMouthButtonClicked}
            >
              Move From Mouth
            </Button>
          </View>
          <View
            style={{
              flex: 2,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%'
            }}
          />
        </>
      )
    }
  }, [
    textFontSize,
    currentDistanceToMouth,
    onDistanceToMouthChange,
    distanceToMouthId,
    moveToMouthButtonClicked,
    moveAwayFromMouthButtonClicked,
    restoreToDefaultButtonClicked
  ])

  // When a face is detected, switch to MoveToMouth
  const faceDetectedCallback = useCallback(() => {
    setLocalMealState(MEAL_STATE.R_MovingToMouth)
    setWaitingText('Waiting to move in front of you...')
  }, [setLocalMealState, setWaitingText])

  // Render the modal body, for calling robot code from within this settings page
  const renderModalBody = useCallback(() => {
    switch (localMealState) {
      case MEAL_STATE.R_MovingToStagingConfiguration:
      case MEAL_STATE.R_MovingFromMouthToStagingConfiguration:
      case MEAL_STATE.R_MovingToMouth:
        return (
          <RobotMotion
            mealState={robotMotionProps.mealState}
            setMealState={robotMotionProps.setMealState}
            nextMealState={robotMotionProps.nextMealState}
            backMealState={robotMotionProps.backMealState}
            actionInput={robotMotionProps.actionInput}
            waitingText={robotMotionProps.waitingText}
          />
        )
      case MEAL_STATE.R_DetectingFace:
        return <DetectingFaceSubcomponent faceDetectedCallback={faceDetectedCallback} />
      default:
        return <></>
    }
  }, [localMealState, robotMotionProps, faceDetectedCallback])

  return (
    <>
      <View
        style={{
          flex: 2,
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          width: '100%'
        }}
      >
        <h5 style={{ textAlign: 'center', fontSize: textFontSize }}>Customize Bite Transfer</h5>
      </View>
      <View
        style={{
          flex: 16,
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          width: '100%'
        }}
      >
        {renderBiteTransferSettings()}
      </View>
      <View
        style={{
          flex: 2,
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          width: '100%'
        }}
      >
        <Button
          variant='success'
          className='mx-2 mb-2 btn-huge'
          size='lg'
          style={{
            fontSize: textFontSize,
            width: '90%',
            height: '90%',
            color: 'black'
          }}
          onClick={doneButtonClicked}
        >
          Done
        </Button>
      </View>
      <Modal
        show={localMealState !== null}
        onHide={() => setLocalMealState(null)}
        size='lg'
        aria-labelledby='contained-modal-title-vcenter'
        backdrop='static'
        keyboard={false}
        centered
        id='robotMotionModal'
        fullscreen={false}
      >
        <Modal.Header closeButton />
        <Modal.Body style={{ overflow: 'hidden' }}>
          <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', width: '90vw', height: '60vh' }}>
            {renderModalBody()}
          </View>
        </Modal.Body>
      </Modal>
    </>
  )
}

export default BiteTransfer
