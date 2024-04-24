// React imports
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useMediaQuery } from 'react-responsive'
import Button from 'react-bootstrap/Button'
import PropTypes from 'prop-types'
import { View } from 'react-native'

// Local imports
import { CAMERA_FEED_TOPIC } from '../Constants'
import TeleopSubcomponent from '../Header/TeleopSubcomponent'
import SettingsPageParent from './SettingsPageParent'
import VideoFeed from '../Home/VideoFeed'

/**
 * The AbovePlate component allows users to configure the "above plate" configuration
 * the robot moves to before bite selection.
 */
const AbovePlate = (props) => {
  // Flag to check if the current orientation is portrait
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  // Indicator of how to arrange screen elements based on orientation
  let dimension = isPortrait ? 'column' : 'row'
  let otherDimension = isPortrait ? 'row' : 'column'
  // Rendering variables
  let textFontSize = 3.5
  let sizeSuffix = isPortrait ? 'vh' : 'vw'

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
          <VideoFeed topic={CAMERA_FEED_TOPIC} updateRateHz={10} webrtcURL={props.webrtcURL} />
        </View>
        <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', width: '100%', height: '100%' }}></View>
        <View style={{ flex: 12, alignItems: 'center', justifyContent: 'center', width: '95%', height: '95%' }}>
          <TeleopSubcomponent />
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
              >
                Move From Resting
              </Button>
            </View>
          </View>
        </View>
      </View>
    )
  }, [dimension, otherDimension, props.webrtcURL, sizeSuffix, textFontSize])

  return (
    <SettingsPageParent title='Above Plate Settings' doneCallback={(e) => console.log(e)}>
      {renderAbovePlateSettings()}
    </SettingsPageParent>
  )
}
AbovePlate.propTypes = {
  // The URL of the webrtc signalling server
  webrtcURL: PropTypes.string.isRequired
}

export default AbovePlate
