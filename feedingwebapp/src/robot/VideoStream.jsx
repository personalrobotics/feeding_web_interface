// React imports
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import axios from 'axios'
import PropTypes from 'prop-types'

// Local imports
import { useROS, subscribeToROSTopic, unsubscribeFromROSTopic } from '../ros/ros_helpers'
import { REALSENSE_WIDTH, REALSENSE_HEIGHT } from '../Pages/Constants'

function createPeer(topic) {
  const peer = new RTCPeerConnection({
    iceServers: [
      {
        urls: 'stun:stun.stunprotocol.org'
      }
    ]
  })
  peer.onnegotiationneeded = () => handleNegotiationNeededEvent(peer, topic)

  return peer
}

async function handleNegotiationNeededEvent(peer, topic) {
  const offer = await peer.createOffer()
  await peer.setLocalDescription(offer)
  const payload = {
    sdp: peer.localDescription
  }
  payload.topic = topic
  console.log('sending payload', payload)
  const { data } = await axios.post('http://localhost:5000/publish', payload)
  const desc = new RTCSessionDescription(data.sdp)
  peer.setRemoteDescription(desc).catch((e) => console.log(e))
}

function VideoStream(props) {
  const canvas = useRef(null)
  const img = useMemo(() => document.createElement('img'), [])

  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  /**
   * Callback for when this page recieves an image from the robot.
   */
  const imageCallback = useCallback(
    (message) => {
        // console.log('Got image message', message)
        img.src = 'data:image/jpg;base64,' + message.data
    },
    [img]
  )

  // Subscribe to the image topic
  useEffect(() => {
    console.log('subscribing to img topic', props.topic)
    let topic = subscribeToROSTopic(ros.current, props.topic, 'sensor_msgs/CompressedImage', imageCallback)
    const cleanup = () => {
      console.log('unsubscribing from img topic', props.topic)
      unsubscribeFromROSTopic(topic, imageCallback)
    }
    window.addEventListener('beforeunload', cleanup)
    /**
     * In practice, because the values passed in in the second argument of
     * useEffect will not change on re-renders, this return statement will
     * only be called when the component unmounts.
     */
    return () => {
      window.removeEventListener('beforeunload', cleanup)
      cleanup()
    }
  }, [imageCallback, props.topic, ros])

  // Draw the image on the canvas.
  const drawImage = useCallback(() => {
    console.log('drawing image')
    if (img.src) {
      // Get the context
      const ctx = canvas.current.getContext('2d')
      // Draw the image
      ctx.drawImage(img, 0, 0, props.width, props.height)
    }
    requestAnimationFrame(drawImage)
  }, [canvas, img, props.width, props.height])

  // Create the WebRTC peer connection. Refresh it every refreshRateHz.
  const [refreshRateDate, setRefreshRateDate] = useState(new Date())
  useEffect(() => {
    // Get the canvas stream
    const stream = canvas.current.captureStream()

    // Create the peer connection
    const peer = createPeer(props.topic)

    // Add the stream to the peer connection
    stream.getTracks().forEach((track) => peer.addTrack(track, stream))

    // Re-run this effect every refreshRateHz
    setTimeout(() => {
      setRefreshRateDate(new Date())
    }, 1000 / props.refreshRateHz)

    // Draw the image on the canvas
    drawImage()

    return () => {
      // TODO: Maybe don't do this given how frequently we refresh?
      peer.close()
    }
  }, [canvas, drawImage, props.refreshRateHz, refreshRateDate, setRefreshRateDate])

  return <canvas ref={canvas} width={props.width} height={props.height} style={{position: 'absolute'}} />
}
VideoStream.propTypes = {
  // The topic to subscribe to
  topic: PropTypes.string.isRequired,
  // How often to refresh the WebRTC connection in Hz
  refreshRateHz: PropTypes.number,
  // The frame rate for the stream
  streamFPS: PropTypes.number.isRequired,
  // The desired width and height of the stream
  width: PropTypes.number.isRequired,
  height: PropTypes.number.isRequired
}
VideoStream.defaultProps = {
  refreshRateHz: 0.2,
  streamFPS: 10,
  width: REALSENSE_WIDTH,
  height: REALSENSE_HEIGHT
}

export default VideoStream
