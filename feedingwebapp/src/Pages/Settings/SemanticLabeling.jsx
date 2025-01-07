// React imports
import React, { useCallback, useMemo, useState } from 'react'
import { View } from 'react-native'

// Local imports
import { useGlobalState, SETTINGS_STATE } from '../GlobalState'
import SettingsPageParent from './SettingsPageParent'

/**
 * The SemanticLabeling component allows users to switch the default user interface
 * to the semanitc labels user interface that allows users to specify the type of 
 * food the robot feeds them using semantic labels.
 */
const SemanticLabeling = () => {
    // Get relevant global state variables
    const setSettingsState = useGlobalState((state) => state.setSettingsState)
    const semanticLabeling = useGlobalState((state) => state.semanticLabeling)
    const setSemanticLabeling = useGlobalState((state) => state.setSemanticLabeling)
    
    // Rendering variables
    let textFontSize = '3.5vh'
    
    // Configure the parameters for SettingsPageParent
    const paramNames = useMemo(() => [], [])
    const [currentParams, setCurrentParams] = useState([])
    
    // Render the settings for the planning scene
    const renderSemanticLabelingSettings = useCallback(() => {
        return (
            <View
                style={{
                flex: 1,
                flexDirection: 'column',
                justifyContent: 'center',
                alignItems: 'center',
                width: '100%',
                height: '100%'
                }}
            >
                <View
                style={{
                    flex: 1,
                    flexDirection: 'row',
                    justifyContent: 'center',
                    alignItems: 'center',
                    width: '100%'
                }}
                >
                <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
                    <input
                    name='semanticLabeling'
                    type='checkbox'
                    checked={semanticLabeling}
                    onChange={(e) => {
                        setSemanticLabeling(e.target.checked)
                    }}
                    style={{ transform: 'scale(2.0)', verticalAlign: 'middle', marginRight: '15px' }}
                    />
                    Semantic Labeling
                </p>
                </View>
            </View>
        )
    }, [semanticLabeling])
    
    return (
        <SettingsPageParent
        title='Semantic Labeling &#9881;'
        doneCallback={() => setSettingsState(SETTINGS_STATE.MAIN)}
        modalShow={false}
        modalOnHide={null}
        modalChildren={null}
        paramNames={paramNames}
        localParamValues={currentParams}
        setLocalParamValues={setCurrentParams}
        >
            {renderSemanticLabelingSettings()}
        </SettingsPageParent>
    )
}