// React Imports
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import PropTypes from 'prop-types'
import { useMediaQuery } from 'react-responsive'
import { View } from 'react-native'

// Local Imports
import {
  useROS,
  createROSTopic,
  createROSMessage,
  createROSActionClient,
  callROSAction,
  destroyActionClient
} from '../../../ros/ros_helpers'
import '../Home.css'
import {
  SERVO_CARTESIAN_TOPIC,
  SERVO_CARTESIAN_TOPIC_MSG,
  SERVO_JOINT_TOPIC,
  SERVO_JOINT_TOPIC_MSG,
  START_SERVO_ACTION_NAME,
  START_SERVO_ACTION_TYPE,
  STOP_SERVO_ACTION_NAME,
  STOP_SERVO_ACTION_TYPE
} from '../../Constants'
import HoldButton from '../../../buttons/HoldButton'

/**
 * The TeleopSubcomponent renders controls for the user to teleoperate the robot,
 * either in cartesian or joint space.
 */
const TeleopSubcomponent = (props) => {
  /**
   * Connect to ROS, if not already connected. Put this in useRef to avoid
   * re-connecting upon re-renders.
   */
  const ros = useRef(useROS().ros)

  /**
   * Create the ROS Action Client to start/stop servo.
   */
  let startServoAction = useMemo(() => {
    return createROSActionClient(ros.current, START_SERVO_ACTION_NAME, START_SERVO_ACTION_TYPE)
  }, [])
  let stopServoAction = useMemo(() => {
    return createROSActionClient(ros.current, STOP_SERVO_ACTION_NAME, STOP_SERVO_ACTION_TYPE)
  }, [])

  /**
   * Create the ROS Topic for cartesian and joint space teleoperation.
   */
  let cartesianTopic = useMemo(() => {
    return createROSTopic(ros.current, SERVO_CARTESIAN_TOPIC, SERVO_CARTESIAN_TOPIC_MSG)
  }, [])
  let jointTopic = useMemo(() => {
    return createROSTopic(ros.current, SERVO_JOINT_TOPIC, SERVO_JOINT_TOPIC_MSG)
  }, [])

  /**
   * When the component is mounted, start servo. Stop it when the component is
   * unmounted.
   */
  useEffect(() => {
    callROSAction(startServoAction, createROSMessage({}))
    return () => {
      callROSAction(stopServoAction, createROSMessage({}))
      destroyActionClient(startServoAction)
      destroyActionClient(stopServoAction)
    }
  }, [startServoAction, stopServoAction])

  /**
   * Callback function to publish constant cartesian cartesian velocity commands.
   *
   * TODO: how should we sync time with the robot?
   * TODO: do we need to include a seq in the message?
   */
  const publishCartesianVelocity = useCallback(
    (x, y, z, rx, ry, rz) => {
      let msg = createROSMessage({
        header: {
          stamp: {
            sec: 0,
            nanosec: 0
          },
          frame_id: 'j2n6s200_link_base'
        },
        twist: {
          linear: {
            x: x,
            y: y,
            z: z
          },
          angular: {
            x: rx,
            y: ry,
            z: rz
          }
        }
      })
      cartesianTopic.publish(msg)
    },
    [cartesianTopic]
  )

  // Render the component
  return (
    <>
      <HoldButton
        rate_hz={10.0}
        holdCallback={() => {
          publishCartesianVelocity(0.0, 0.0, 0.1, 0.0, 0.0, 0.0)
        }}
        cleanupCallback={() => {
          publishCartesianVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        }}
        buttonStyle={{}}
      >
        Up
      </HoldButton>
    </>
  )
}
TeleopSubcomponent.propTypes = {}
export default TeleopSubcomponent
