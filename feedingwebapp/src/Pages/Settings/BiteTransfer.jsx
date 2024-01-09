// React imports
import React, { useCallback, useEffect, useRef, useState } from 'react'
import { useId, Label, SpinButton } from '@fluentui/react-components'
import Button from 'react-bootstrap/Button'
import { View } from 'react-native'

// Local imports
import { useROS, createROSService, createROSServiceRequest, getParameterValue } from '../../ros/ros_helpers'
import {
  GET_PARAMETERS_SERVICE_NAME,
  GET_PARAMETERS_SERVICE_TYPE,
  SET_PARAMETERS_SERVICE_NAME,
  SET_PARAMETERS_SERVICE_TYPE
} from '../Constants'
import { useGlobalState, SETTINGS_STATE } from '../GlobalState'

/**
 * The BiteTransfer component allows users to configure parameters related to the
 * bite transfer.
 */
const BiteTransfer = () => {
  // Get relevant global state variables
  const setSettingsState = useGlobalState((state) => state.setSettingsState)

  // Create relevant local state variables
  const [currentDistanceToMouth, setCurrentDistanceToMouth] = useState(null)

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

  // Callback to return to the main settings page
  const doneButtonClicked = useCallback(() => {
    setSettingsState(SETTINGS_STATE.MAIN)
  }, [setSettingsState])

  // Callback for when the user changes the distance to mouth
  const onDistanceToMouthChange = useCallback(
    (_ev, data) => {
      let value = data.value ? data.value : parseFloat(data.displayValue)
      let fullDistanceToMouth = [value / 100.0, currentDistanceToMouth[1], currentDistanceToMouth[2]]
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
    [setParametersService, setCurrentDistanceToMouth, currentDistanceToMouth]
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
              flex: 1,
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%'
            }}
          >
            {/* <h5 style={{ textAlign: 'center', fontSize: textFontSize }}>Distance to Mouth</h5>
            <h5 style={{ textAlign: 'center', fontSize: textFontSize }}>{currentDistanceToMouth[0]*100}</h5> */}
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
          </View>
        </>
      )
    }
  }, [textFontSize, currentDistanceToMouth, onDistanceToMouthChange, distanceToMouthId])

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
    </>
  )
}

export default BiteTransfer
