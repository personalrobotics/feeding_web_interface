// React imports
import React, { useCallback, useEffect, useMemo, useState } from 'react'
import { useMediaQuery } from 'react-responsive'
import PropTypes from 'prop-types'
import { useId, Label, SpinButton } from '@fluentui/react-components'
import Button from 'react-bootstrap/Button'
import { View } from 'react-native'

// Local imports
import { getRobotMotionText, DISTANCE_TO_MOUTH_PARAM } from '../Constants'
import { useGlobalState, MEAL_STATE, SETTINGS_STATE } from '../GlobalState'
import RobotMotion from '../Home/MealStates/RobotMotion'
import DetectingFaceSubcomponent from '../Home/MealStates/DetectingFaceSubcomponent'
import SettingsPageParent from './SettingsPageParent'

/**
 * The BiteTransfer component allows users to configure parameters related to the
 * bite transfer.
 */
const BiteTransfer = (props) => {
  // Get relevant global state variables
  const setSettingsState = useGlobalState((state) => state.setSettingsState)
  const globalMealState = useGlobalState((state) => state.mealState)
  const setPaused = useGlobalState((state) => state.setPaused)
  const biteTransferPageAtFace = useGlobalState((state) => state.biteTransferPageAtFace)
  const setBiteTransferPageAtFace = useGlobalState((state) => state.setBiteTransferPageAtFace)

  // Create relevant local state variables
  // Configure the parameters for SettingsPageParent
  const paramNames = useMemo(() => [DISTANCE_TO_MOUTH_PARAM], [])
  const [currentDistanceToMouth, setCurrentDistanceToMouth] = useState([null])
  const [localCurrAndNextMealState, setLocalCurrAndNextMealState] = useState(
    globalMealState === MEAL_STATE.U_BiteDone || globalMealState === MEAL_STATE.R_DetectingFace || biteTransferPageAtFace
      ? [MEAL_STATE.R_MovingFromMouth, null]
      : [MEAL_STATE.R_MovingToStagingConfiguration, null]
  )
  const actionInput = useMemo(() => ({}), [])
  const [doneButtonIsClicked, setDoneButtonIsClicked] = useState(false)

  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Indicator of how to arrange screen elements based on orientation
  let dimension = isPortrait ? 'column' : 'row'
  // Rendering variables
  let textFontSize = '3.5vh'

  // Get min and max distance to mouth
  const minDistanceToMouth = 1 // cm
  const maxDistanceToMouth = 10 // cm

  // When we set local meal state, also update bite transfer page at face
  const setLocalCurrMealStateWrapper = useCallback(
    (newLocalCurrMealState, newLocalNextMealState = null) => {
      let oldLocalCurrMealState = localCurrAndNextMealState[0]
      // If the oldlocalCurrMealState was R_MovingToMouth, then the robot is at the mouth
      setBiteTransferPageAtFace(oldLocalCurrMealState === MEAL_STATE.R_MovingToMouth)
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
    [localCurrAndNextMealState, setLocalCurrAndNextMealState, setBiteTransferPageAtFace, doneButtonIsClicked, setPaused, setSettingsState]
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

  // The first time the page is rendered, get the current distance to mouth
  useEffect(() => {
    setDoneButtonIsClicked(false)
    // Start in a moving state, not a paused state
    setPaused(false)
  }, [setPaused, setDoneButtonIsClicked])

  // Callback to move the robot to the mouth
  const moveToMouthButtonClicked = useCallback(() => {
    setLocalCurrMealStateWrapper(MEAL_STATE.R_DetectingFace)
    setDoneButtonIsClicked(false)
  }, [setLocalCurrMealStateWrapper, setDoneButtonIsClicked])

  // Callback to move the robot away from the mouth
  const moveAwayFromMouthButtonClicked = useCallback(() => {
    setLocalCurrMealStateWrapper(MEAL_STATE.R_MovingFromMouth)
    setDoneButtonIsClicked(false)
  }, [setLocalCurrMealStateWrapper, setDoneButtonIsClicked])

  // Callback to return to the main settings page
  const doneButtonClicked = useCallback(() => {
    setDoneButtonIsClicked(true)
    // Determine the state to move to based on the state before entering settings
    let localCurrMealState = MEAL_STATE.R_MovingFromMouth
    let localNextMealState
    // To get to Settings, the globalMealState must be one of the NON_MOVING_STATES
    switch (globalMealState) {
      case MEAL_STATE.U_BiteDone:
        if (biteTransferPageAtFace) {
          localCurrMealState = null
          localNextMealState = null
        } else {
          localNextMealState = MEAL_STATE.R_DetectingFace
        }
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
  }, [biteTransferPageAtFace, globalMealState, setLocalCurrMealStateWrapper, setDoneButtonIsClicked])

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
      let fullDistanceToMouth = [value / 100.0, currentDistanceToMouth[0][1], currentDistanceToMouth[0][2]]
      setCurrentDistanceToMouth([fullDistanceToMouth])
    },
    [currentDistanceToMouth, minDistanceToMouth, maxDistanceToMouth]
  )

  // Callback to render the main contents of the page
  const distanceToMouthId = useId()
  const renderBiteTransferSettings = useCallback(() => {
    if (currentDistanceToMouth[0] === null) {
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
          <View
            style={{
              flex: 8,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%',
              zIndex: 1
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
              value={currentDistanceToMouth[0][0] * 100}
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
          </View>
          <View
            style={{
              flex: 8,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%',
              zIndex: 1
            }}
          >
            <View
              style={{
                flex: 1,
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
                flex: 1,
                justifyContent: 'center',
                alignItems: 'center',
                width: '100%'
              }}
            />
            <View
              style={{
                flex: 1,
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
                flex: 1,
                justifyContent: 'center',
                alignItems: 'center',
                width: '100%'
              }}
            />
          </View>
        </View>
      )
    }
  }, [
    dimension,
    textFontSize,
    currentDistanceToMouth,
    onDistanceToMouthChange,
    distanceToMouthId,
    moveToMouthButtonClicked,
    moveAwayFromMouthButtonClicked
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
      title='Bite Transfer Settings'
      doneCallback={doneButtonClicked}
      modalShow={localCurrAndNextMealState[0] !== null}
      modalOnHide={() => setLocalCurrMealStateWrapper(null)}
      modalChildren={renderModalBody()}
      paramNames={paramNames}
      localParamValues={currentDistanceToMouth}
      setLocalParamValues={setCurrentDistanceToMouth}
    >
      {renderBiteTransferSettings()}
    </SettingsPageParent>
  )
}
BiteTransfer.propTypes = {
  // The URL of the webrtc signalling server
  webrtcURL: PropTypes.string.isRequired
}

export default BiteTransfer
