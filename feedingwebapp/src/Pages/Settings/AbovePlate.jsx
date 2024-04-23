// React imports
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useMediaQuery } from 'react-responsive'
import PropTypes from 'prop-types'
import { View } from 'react-native'

// Local imports
import { CAMERA_FEED_TOPIC } from '../Constants'
import TeleopSubcomponent from '../Header/TeleopSubcomponent'
import VideoFeed from '../Home/VideoFeed'

/**
 * The AbovePlate component allows users to configure the "above plate" configuration
 * the robot moves to before bite selection.
 */
const AbovePlate = (props) => {
  // Rendering variables
  let textFontSize = '3.5vh'
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Indicator of how to arrange screen elements based on orientation
  let dimension = isPortrait ? 'column' : 'row'

  return (
    <View
      style={{
        flex: 1,
        flexDirection: 'col',
        alignItems: 'center',
        justifyContent: 'center',
        width: '100%',
        height: '100%'
      }}
    >
        <View
        style={{
            flex: 1,
            alignItems: 'center',
            justifyContent: 'center',
            width: '100%',
            height: '100%'
        }}
        >
            <p style={{ textAlign: 'center', fontSize: textFontSize, margin: 0 }} className='txt-huge'>
                Above Plate Settings
            </p>
        </View>
        <View
        style={{
            flex: 9,
            flexDirection: dimension,
            alignItems: 'center',
            justifyContent: 'center',
            width: '100%',
            height: '100%'
        }}
        >
        <View style={{ flex: 3, alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}>
            <VideoFeed topic={CAMERA_FEED_TOPIC} updateRateHz={10} webrtcURL={props.webrtcURL} />
        </View>
        <View style={{ flex: 7, alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}>
            <TeleopSubcomponent />
        </View>
        {/* TODO: add 4 buttons, to move to/from staging configuration and to/from resting configuration */}
        </View>
    </View>
  )
}
AbovePlate.propTypes = {
  // The URL of the webrtc signalling server
  webrtcURL: PropTypes.string.isRequired
}

export default AbovePlate
