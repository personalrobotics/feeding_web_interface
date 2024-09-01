// React imports
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useMediaQuery } from 'react-responsive'
import ButtonGroup from 'react-bootstrap/ButtonGroup'
import Dropdown from 'react-bootstrap/Dropdown'
import DropdownButton from 'react-bootstrap/DropdownButton'
import { View } from 'react-native'

// Local imports
import { PLANNING_SCENE_PARAM, PLANNING_SCENE_GET_PARAMETERS_SERVICE_NAME, PLANNING_SCENE_GET_PARAMETERS_SERVICE_TYPE } from '../Constants'
import { useGlobalState, SETTINGS_STATE } from '../GlobalState'
import { useROS, createROSService, createROSServiceRequest, getValueFromParameter } from '../../ros/ros_helpers'
import SettingsPageParent from './SettingsPageParent'

/**
 * The PlanningScene component allows users to change the planning scene that a particular
 * settings namespace is configured with
 */
const PlanningScene = () => {
  // Get relevant global state variables
  const setSettingsState = useGlobalState((state) => state.setSettingsState)

  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Indicator of how to arrange screen elements based on orientation
  let dimension = isPortrait ? 'column' : 'row'
  // Rendering variables
  let textFontSize = '3.5vh'

  // Configure the parameters for SettingsPageParent
  const paramNames = useMemo(() => [PLANNING_SCENE_PARAM], [])
  const [currentParams, setCurrentParams] = useState([null])

  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  // Get the options for the planning scene
  let getParametersService = useRef(
    createROSService(ros.current, PLANNING_SCENE_GET_PARAMETERS_SERVICE_NAME, PLANNING_SCENE_GET_PARAMETERS_SERVICE_TYPE)
  )
  const [planningSceneNamespaces, setPlanningSceneNamespaces] = useState([])
  useEffect(() => {
    let service = getParametersService.current
    let request = createROSServiceRequest({
      names: ['namespaces']
    })
    service.callService(request, (response) => {
      if (response.values.length > 0 && response.values[0].type === 9) {
        setPlanningSceneNamespaces(getValueFromParameter(response.values[0]))
      } else {
        console.error('PlanningScene: Error getting planning scene namespaces')
      }
    })
  }, [getParametersService, setPlanningSceneNamespaces])

  // Render the settings for the planning scene
  const renderPlanningSceneSettings = useCallback(() => {
    if (currentParams.some((param) => param === null)) {
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
          <DropdownButton
            as={ButtonGroup}
            key='planningSceneOptions'
            id={`dropdown-button-drop-down`}
            drop='down'
            variant='secondary'
            title={currentParams[0]}
            size='lg'
          >
            {planningSceneNamespaces.map((namespace) => (
              <Dropdown.Item key={namespace} onClick={() => setCurrentParams([namespace])} active={namespace === currentParams[0]}>
                {namespace}
              </Dropdown.Item>
            ))}
          </DropdownButton>
        </View>
      )
    }
  }, [currentParams, dimension, planningSceneNamespaces, setCurrentParams, textFontSize])

  return (
    <SettingsPageParent
      title='Planning Scene &#9881;'
      doneCallback={() => setSettingsState(SETTINGS_STATE.MAIN)}
      modalShow={false}
      modalOnHide={null}
      modalChildren={null}
      paramNames={paramNames}
      localParamValues={currentParams}
      setLocalParamValues={setCurrentParams}
    >
      {renderPlanningSceneSettings()}
    </SettingsPageParent>
  )
}

export default PlanningScene
