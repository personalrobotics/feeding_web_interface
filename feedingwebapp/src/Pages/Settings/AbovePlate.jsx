// React imports
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useMediaQuery } from 'react-responsive'
import Button from 'react-bootstrap/Button'
import Dropdown from 'react-bootstrap/Dropdown'
import SplitButton from 'react-bootstrap/SplitButton'
import PropTypes from 'prop-types'
import { View } from 'react-native'

// Local imports
import { useROS, createROSService, createROSServiceRequest, getParameterValue } from '../../ros/ros_helpers'
import {
  ABOVE_PLATE_PARAM,
  CAMERA_FEED_TOPIC,
  getRobotMotionText,
  GET_JOINT_STATE_SERVICE_NAME,
  GET_JOINT_STATE_SERVICE_TYPE,
  GET_PARAMETERS_SERVICE_NAME,
  GET_PARAMETERS_SERVICE_TYPE,
  ROBOT_JOINTS,
  SET_PARAMETERS_SERVICE_NAME,
  SET_PARAMETERS_SERVICE_TYPE
} from '../Constants'
import { useGlobalState, MEAL_STATE, SETTINGS_STATE, DEFAULT_NAMESPACE } from '../GlobalState'
import RobotMotion from '../Home/MealStates/RobotMotion'
import DetectingFaceSubcomponent from '../Home/MealStates/DetectingFaceSubcomponent'
import TeleopSubcomponent from '../Header/TeleopSubcomponent'
import SettingsPageParent from './SettingsPageParent'
import VideoFeed from '../Home/VideoFeed'

/**
 * The AbovePlate component allows users to configure the "above plate" configuration
 * the robot moves to before bite selection.
 */
const AbovePlate = (props) => {
  // Get relevant global state variables
  const settingsPresets = useGlobalState((state) => state.settingsPresets)
  const setSettingsState = useGlobalState((state) => state.setSettingsState)
  const globalMealState = useGlobalState((state) => state.mealState)
  const setPaused = useGlobalState((state) => state.setPaused)
  const biteTransferPageAtFace = useGlobalState((state) => state.biteTransferPageAtFace)

  // Create relevant local state variables
  const [currentAbovePlateParam, setCurrentAbovePlateParam] = useState(null)
  const [localCurrAndNextMealState, setLocalCurrAndNextMealState] = useState(
    globalMealState === MEAL_STATE.U_BiteDone || biteTransferPageAtFace
      ? [MEAL_STATE.R_MovingFromMouth, MEAL_STATE.R_MovingAbovePlate]
      : [MEAL_STATE.R_MovingAbovePlate, null]
  )
  const actionInput = useMemo(() => {
    if (localCurrAndNextMealState[0] === MEAL_STATE.R_MovingToConfigurationAbovePlate) {
      return {
        joint_positions: currentAbovePlateParam
      }
    }
    return {}
  }, [currentAbovePlateParam, localCurrAndNextMealState])
  const [doneButtonIsClicked, setDoneButtonIsClicked] = useState(false)
  const [robotIsAbovePlate, setRobotIsAbovePlate] = useState(true)
  const [zoomLevel, setZoomLevel] = useState(1.0)

  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Indicator of how to arrange screen elements based on orientation
  let dimension = isPortrait ? 'column' : 'row'
  let otherDimension = isPortrait ? 'row' : 'column'
  // Rendering variables
  let textFontSize = 3.5
  let sizeSuffix = isPortrait ? 'vh' : 'vw'

  // Update other state variables that are related to the local meal state
  const setLocalCurrMealStateWrapper = useCallback(
    (newLocalCurrMealState, newLocalNextMealState = null) => {
      let oldLocalCurrMealState = localCurrAndNextMealState[0]
      // If the oldlocalCurrMealState was R_MovingToMouth, then the robot is at the mouth
      setRobotIsAbovePlate(
        oldLocalCurrMealState === MEAL_STATE.R_MovingAbovePlate || oldLocalCurrMealState === MEAL_STATE.R_MovingToConfigurationAbovePlate
      )
      // Start in a moving state, not a paused state
      setPaused(false)
      if (newLocalCurrMealState === null && doneButtonIsClicked) {
        // After the done button is clicked, the robot may have to do up to two
        // motions to restore itself to its old state. After that, this goes
        // back to the main settings page.
        setSettingsState(SETTINGS_STATE.MAIN)
      } else {
        setLocalCurrAndNextMealState([newLocalCurrMealState, newLocalNextMealState])
      }
    },
    [localCurrAndNextMealState, setLocalCurrAndNextMealState, setRobotIsAbovePlate, doneButtonIsClicked, setPaused, setSettingsState]
  )

  // Store the props for the RobotMotion call.
  const robotMotionProps = useMemo(() => {
    let localCurrMealState = localCurrAndNextMealState[0]
    let localNextMealState = localCurrAndNextMealState[1]
    return {
      mealState: localCurrMealState,
      setMealState: setLocalCurrMealStateWrapper,
      nextMealState: localNextMealState,
      backMealState: null,
      actionInput: actionInput,
      waitingText: localCurrMealState ? getRobotMotionText(localCurrMealState) : ''
    }
  }, [localCurrAndNextMealState, setLocalCurrMealStateWrapper, actionInput])

  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  /**
   * Create the ROS Service Clients to get/set parameters.
   */
  let getJointStateService = useRef(createROSService(ros.current, GET_JOINT_STATE_SERVICE_NAME, GET_JOINT_STATE_SERVICE_TYPE))
  let getParametersService = useRef(createROSService(ros.current, GET_PARAMETERS_SERVICE_NAME, GET_PARAMETERS_SERVICE_TYPE))
  let setParametersService = useRef(createROSService(ros.current, SET_PARAMETERS_SERVICE_NAME, SET_PARAMETERS_SERVICE_TYPE))

  // Reset state the first time the page is rendered
  useEffect(() => {
    setDoneButtonIsClicked(false)
    // Start in a moving state, not a paused state
    setPaused(false)
    let service = getParametersService.current
    // First, attempt to get the current above plate param
    let currentRequest = createROSServiceRequest({
      names: [settingsPresets.current.concat('.', ABOVE_PLATE_PARAM)]
    })
    service.callService(currentRequest, (response) => {
      console.log('Got current above plate param response', response)
      if (response.values.length === 0 || response.values[0].type === 0) {
        // Parameter not set
        // Second, attempt to get the default distance to mouth
        let defaultRequest = createROSServiceRequest({
          names: [DEFAULT_NAMESPACE.concat('.', ABOVE_PLATE_PARAM)]
        })
        service.callService(defaultRequest, (response) => {
          console.log('Got default above plate param response', response)
          if (response.values.length > 0 && response.values[0].type === 8) {
            setCurrentAbovePlateParam(getParameterValue(response.values[0]))
          }
        })
      } else {
        setCurrentAbovePlateParam(getParameterValue(response.values[0]))
      }
    })
  }, [settingsPresets, setPaused, getParametersService, setCurrentAbovePlateParam, setLocalCurrMealStateWrapper])

  // Callback to set the above plate parameter
  const setAbovePlateParam = useCallback(
    (jointStates) => {
      let service = setParametersService.current
      let request = createROSServiceRequest({
        parameters: [
          {
            name: settingsPresets.current.concat('.', ABOVE_PLATE_PARAM),
            value: {
              type: 8, // double array
              double_array_value: jointStates
            }
          }
        ]
      })
      service.callService(request, (response) => {
        console.log('Got response from setting parameter', response)
      })
    },
    [setParametersService, settingsPresets]
  )

  // Callback to restore the above plate param to a specified preset
  const restoreToPreset = useCallback(
    (preset) => {
      console.log('restoreToPreset called with', preset)
      let service = getParametersService.current
      // Attempt to get the preset above plate param
      let request = createROSServiceRequest({
        names: [preset.concat('.', ABOVE_PLATE_PARAM)]
      })
      service.callService(request, (response) => {
        console.log('Got above plate param from preset', preset, 'response', response)
        if (response.values.length > 0 && response.values[0].type === 8) {
          setCurrentAbovePlateParam(getParameterValue(response.values[0]))
          setLocalCurrMealStateWrapper(MEAL_STATE.R_MovingToConfigurationAbovePlate)
        } else {
          restoreToPreset(DEFAULT_NAMESPACE)
        }
      })
    },
    [getParametersService, setLocalCurrMealStateWrapper, setCurrentAbovePlateParam]
  )

  // Get the current joint states and store them as the above plate param
  const storeJointStatesAsLocalParam = useCallback(() => {
    let service = getJointStateService.current
    let request = createROSServiceRequest({
      names: ROBOT_JOINTS
    })
    service.callService(request, (response) => {
      console.log('Got joint state response', response)
      setAbovePlateParam(response.joint_positions)
    })
  }, [getJointStateService, setAbovePlateParam])

  // Callback to move the robot to the staging configuration
  const moveToStagingButtonClicked = useCallback(() => {
    if (robotIsAbovePlate) {
      storeJointStatesAsLocalParam()
    }
    setDoneButtonIsClicked(false)
    setLocalCurrMealStateWrapper(MEAL_STATE.R_MovingToStagingConfiguration)
  }, [setLocalCurrMealStateWrapper, setDoneButtonIsClicked, storeJointStatesAsLocalParam, robotIsAbovePlate])

  // Callback to move the robot away from the staging configuration
  const moveFromStagingButtonClicked = useCallback(() => {
    setDoneButtonIsClicked(false)
    setLocalCurrMealStateWrapper(MEAL_STATE.R_MovingToConfigurationAbovePlate)
  }, [setLocalCurrMealStateWrapper, setDoneButtonIsClicked])

  // Callback to move the robot to the resting configuration
  const moveToRestingButtonClicked = useCallback(() => {
    if (robotIsAbovePlate) {
      storeJointStatesAsLocalParam()
    }
    setDoneButtonIsClicked(false)
    setLocalCurrMealStateWrapper(MEAL_STATE.R_MovingToRestingPosition)
  }, [setLocalCurrMealStateWrapper, setDoneButtonIsClicked, storeJointStatesAsLocalParam, robotIsAbovePlate])

  // Callback to move the robot away from the resting configuration
  const moveFromRestingButtonClicked = useCallback(() => {
    setDoneButtonIsClicked(false)
    setLocalCurrMealStateWrapper(MEAL_STATE.R_MovingToConfigurationAbovePlate)
  }, [setLocalCurrMealStateWrapper, setDoneButtonIsClicked])

  // Callback to return to the main settings page
  const doneButtonClicked = useCallback(() => {
    setDoneButtonIsClicked(true)
    // Determine the state to move to based on the state before entering settings
    let localCurrMealState
    let localNextMealState = null
    // To get to Settings, the globalMealState must be one of the NON_MOVING_STATES
    switch (globalMealState) {
      case MEAL_STATE.U_BiteDone:
        localCurrMealState = MEAL_STATE.R_MovingToStagingConfiguration
        localNextMealState = MEAL_STATE.R_DetectingFace
        break
      case MEAL_STATE.U_PreMeal:
      case MEAL_STATE.U_BiteSelection:
        localNextMealState = MEAL_STATE.R_MovingAbovePlate
        break
      case MEAL_STATE.U_BiteAcquisitionCheck:
        localNextMealState = MEAL_STATE.R_MovingToRestingPosition
        break
      case MEAL_STATE.R_DetectingFace:
        localNextMealState = MEAL_STATE.R_MovingToStagingConfiguration
        break
      case MEAL_STATE.U_PostMeal:
        localNextMealState = MEAL_STATE.R_StowingArm
        break
      default:
        localNextMealState = MEAL_STATE.R_MovingAbovePlate
        break
    }
    setLocalCurrMealStateWrapper(localCurrMealState, localNextMealState)
  }, [globalMealState, setLocalCurrMealStateWrapper, setDoneButtonIsClicked])

  // Callback to render the main contents of the page
  const renderAbovePlateSettings = useCallback(() => {
    return (
      <View
        style={{
          flex: 1,
          flexDirection: dimension,
          justifyContent: 'center',
          alignItems: 'center',
          width: '100%',
          height: '100%'
        }}
      >
        <View style={{ flex: 8, alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}>
          <VideoFeed topic={CAMERA_FEED_TOPIC} updateRateHz={10} webrtcURL={props.webrtcURL} zoom={zoomLevel} setZoom={setZoomLevel} />
        </View>
        <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}></View>
        <View style={{ flex: 12, flexDirection: 'column', alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}>
          {localCurrAndNextMealState[0] === null ? (
            <>
              <View style={{ flex: 5, alignItems: 'center', justifyContent: 'center', width: '95%', height: '95%' }}>
                <TeleopSubcomponent />
              </View>
              <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}>
                <SplitButton
                  variant='warning'
                  className='mx-2 mb-2 btn-huge'
                  size='lg'
                  style={{
                    fontSize: (textFontSize * 0.5).toString() + sizeSuffix,
                    width: '60%',
                    color: 'black'
                  }}
                  title={'Set to '.concat(DEFAULT_NAMESPACE)}
                  onClick={() => restoreToPreset(DEFAULT_NAMESPACE)}
                >
                  <Dropdown.Item key={DEFAULT_NAMESPACE} onClick={() => restoreToPreset(DEFAULT_NAMESPACE)}>
                    Set to {DEFAULT_NAMESPACE}
                  </Dropdown.Item>
                  {settingsPresets.customNames.map((preset) => (
                    <Dropdown.Item key={preset} onClick={() => restoreToPreset(preset)}>
                      Set to {preset}
                    </Dropdown.Item>
                  ))}
                </SplitButton>
              </View>
            </>
          ) : (
            <></>
          )}
        </View>
        <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}></View>
        <View
          style={{
            flex: 4,
            flexDirection: otherDimension,
            alignItems: 'center',
            justifyContent: 'center',
            width: '100%',
            height: '100%'
          }}
        >
          <View style={{ flex: 1, flexDirection: 'column', alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}>
            <View
              style={{ flex: 1, flexDirection: 'column', alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}
            >
              <Button
                variant='warning'
                className='mx-2 mb-2 btn-huge'
                size='lg'
                style={{
                  fontSize: (textFontSize * 0.5).toString() + sizeSuffix,
                  width: '90%',
                  height: '90%',
                  color: 'black'
                }}
                onClick={moveToStagingButtonClicked}
              >
                Move To Staging
              </Button>
            </View>
            <View
              style={{ flex: 1, flexDirection: 'column', alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}
            >
              <Button
                variant='warning'
                className='mx-2 mb-2 btn-huge'
                size='lg'
                style={{
                  fontSize: (textFontSize * 0.5).toString() + sizeSuffix,
                  width: '90%',
                  height: '90%',
                  color: 'black'
                }}
                onClick={moveFromStagingButtonClicked}
              >
                Move From Staging
              </Button>
            </View>
          </View>
          <View style={{ flex: 1, flexDirection: 'column', alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}>
            <View
              style={{ flex: 1, flexDirection: 'column', alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}
            >
              <Button
                variant='warning'
                className='mx-2 mb-2 btn-huge'
                size='lg'
                style={{
                  fontSize: (textFontSize * 0.5).toString() + sizeSuffix,
                  width: '90%',
                  height: '90%',
                  color: 'black'
                }}
                onClick={moveToRestingButtonClicked}
              >
                Move To Resting
              </Button>
            </View>
            <View
              style={{ flex: 1, flexDirection: 'column', alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}
            >
              <Button
                variant='warning'
                className='mx-2 mb-2 btn-huge'
                size='lg'
                style={{
                  fontSize: (textFontSize * 0.5).toString() + sizeSuffix,
                  width: '90%',
                  height: '90%',
                  color: 'black'
                }}
                onClick={moveFromRestingButtonClicked}
              >
                Move From Resting
              </Button>
            </View>
          </View>
        </View>
      </View>
    )
  }, [
    dimension,
    otherDimension,
    textFontSize,
    sizeSuffix,
    zoomLevel,
    localCurrAndNextMealState,
    props.webrtcURL,
    settingsPresets,
    restoreToPreset,
    moveToStagingButtonClicked,
    moveFromStagingButtonClicked,
    moveToRestingButtonClicked,
    moveFromRestingButtonClicked
  ])

  // When a face is detected, switch to MoveToMouth
  const faceDetectedCallback = useCallback(() => {
    setLocalCurrMealStateWrapper(MEAL_STATE.R_MovingToMouth)
  }, [setLocalCurrMealStateWrapper])

  // Render the modal body, for calling robot code from within this settings page
  const renderModalBody = useCallback(() => {
    let localCurrMealState = localCurrAndNextMealState[0]
    switch (localCurrMealState) {
      case MEAL_STATE.R_MovingToStagingConfiguration:
      case MEAL_STATE.R_MovingFromMouth:
      case MEAL_STATE.R_MovingToMouth:
      case MEAL_STATE.R_MovingAbovePlate:
      case MEAL_STATE.R_MovingToRestingPosition:
      case MEAL_STATE.R_StowingArm:
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
        return <DetectingFaceSubcomponent faceDetectedCallback={faceDetectedCallback} webrtcURL={props.webrtcURL} />
      default:
        return <></>
    }
  }, [localCurrAndNextMealState, props.webrtcURL, robotMotionProps, faceDetectedCallback])

  return (
    <SettingsPageParent
      title='Above Plate Settings'
      doneCallback={doneButtonClicked}
      modalShow={localCurrAndNextMealState[0] !== null}
      modalOnHide={() => setLocalCurrMealStateWrapper(null)}
      modalChildren={renderModalBody()}
    >
      {renderAbovePlateSettings()}
    </SettingsPageParent>
  )
}
AbovePlate.propTypes = {
  // The URL of the webrtc signalling server
  webrtcURL: PropTypes.string.isRequired
}

export default AbovePlate
