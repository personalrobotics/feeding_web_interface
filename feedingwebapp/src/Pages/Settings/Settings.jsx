// React imports
import React, { useCallback } from 'react'
import { View } from 'react-native'

// Local imports
import { useGlobalState, SETTINGS_STATE } from '../GlobalState'
import Main from './Main'
import BiteTransfer from './BiteTransfer'

/**
 * The Settings components displays the appropriate settings page based on the
 * current settings state.
 */
const Settings = () => {
  // Get the relevant values from global state
  const settingsState = useGlobalState((state) => state.settingsState)

  const getComponentBySettingsState = useCallback(() => {
    console.log('getComponentBySettingsState', settingsState)
    switch (settingsState) {
      case SETTINGS_STATE.MAIN:
        return <Main />
      case SETTINGS_STATE.BITE_TRANSFER:
        return <BiteTransfer />
      default:
        console.log('Invalid settings state', settingsState)
        return <Main />
    }
  }, [settingsState])

  // Render the component
  return (
    <View style={{ flex: 1, alignItems: 'center', justifyContent: 'start' }}>
      {/**
       * The main contents of the screen depends on the settingsState.
       */}
      {getComponentBySettingsState()}
    </View>
  )
}

export default Settings
