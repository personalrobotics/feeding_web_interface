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
  // Get local state
  const CARTESIAN_MODE = 'cartesian'
  const JOINT_MODE = 'joint'
  const [teleopMode, setTeleopMode] = useState(CARTESIAN_MODE)

  // Cartesian teleop speeds
  const LINEAR_SPEED = 0.1 // m/s
  const ANGULAR_SPEED = 0.3 // rad/s
  const JOINT_SPEED = 0.5 // rad/s

  // Style configuration
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  let dimension = isPortrait ? 'column' : 'row'
  let textFontSize = isPortrait ? '3vh' : '2.5vw'
  const buttonStyle = useMemo(() => {
    return {
      width: '100%',
      height: '100%',
      border: '1px solid black',
      padding: '0rem',
      borderRadius: '0rem'
    }
  }, [])

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
      callROSAction(stopServoAction, createROSMessage({}), null, () => {
        destroyActionClient(startServoAction)
        destroyActionClient(stopServoAction)
      })
    }
  }, [startServoAction, stopServoAction])

  /**
   * Callback function to publish constant cartesian cartesian velocity commands.
   */
  const publishCartesianVelocity = useCallback(
    (x, y, z, rx, ry, rz) => {
      let msg = createROSMessage({
        header: {
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

  /**
   * Callback function to publish a JointJog command to move a single joint.
   */
  const publishJointJog = useCallback(
    (joint, velocity) => {
      let msg = createROSMessage({
        header: {
          frame_id: 'j2n6s200_link_base'
        },
        joint_names: [joint],
        velocities: [JOINT_SPEED * velocity]
      })
      jointTopic.publish(msg)
    },
    [jointTopic, JOINT_SPEED]
  )

  /**
   * Arrange the specified 6 elements in a trackpad-like arrangement.
   *
   * e11 and e12 are the "first-level," opposite from one another.
   * e21, and e22 are the "second-level," opposite from one another.
   * e31 and e32 are the "third-level," opposite from one another.
   */
  const trackpadArrangement = useCallback((e11, e12, e21, e22, e31, e32, horizontal = true) => {
    let mainFlexDirection = horizontal ? 'row' : 'col'
    let subFlexDirection = horizontal ? 'col' : 'row'

    return (
      <>
        <View
          style={{
            flex: 1,
            flexDirection: mainFlexDirection,
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%',
            height: '100%'
          }}
        >
          <View
            style={{
              flex: 1,
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%',
              height: '100%'
            }}
          >
            {e11}
          </View>
          <View
            style={{
              flex: 4,
              flexDirection: subFlexDirection,
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%',
              height: '100%'
            }}
          >
            <View
              style={{
                flex: 1,
                justifyContent: 'center',
                alignItems: 'center',
                width: '100%',
                height: '100%'
              }}
            >
              {e21}
            </View>
            <View
              style={{
                flex: 2,
                flexDirection: mainFlexDirection,
                justifyContent: 'center',
                alignItems: 'center',
                width: '100%',
                height: '100%'
              }}
            >
              <View
                style={{
                  flex: 2,
                  justifyContent: 'center',
                  alignItems: 'center',
                  width: '100%',
                  height: '100%'
                }}
              >
                {e31}
              </View>
              <View
                style={{
                  flex: 2,
                  justifyContent: 'center',
                  alignItems: 'center',
                  width: '100%',
                  height: '100%'
                }}
              >
                {e32}
              </View>
            </View>
            <View
              style={{
                flex: 1,
                justifyContent: 'center',
                alignItems: 'center',
                width: '100%',
                height: '100%'
              }}
            >
              {e22}
            </View>
          </View>
          <View
            style={{
              flex: 1,
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%',
              height: '100%'
            }}
          >
            {e12}
          </View>
        </View>
      </>
    )
  }, [])

  /**
   * Callback function to generate a button array of size major_axis*minor_axis and
   * fill it with specified elements row-wise.
   */
  const arrayArrangement = useCallback((major_axis, minor_axis, elements, horizontal = true) => {
    let majorFlexDirection = horizontal ? 'row' : 'col'
    let minorFlexDirection = horizontal ? 'col' : 'row'

    let layout = []
    for (let i = 0; i < major_axis; i++) {
      let row = []
      for (let j = 0; j < minor_axis; j++) {
        row.push(
          <View
            style={{
              flex: 1,
              justifyContent: 'center',
              alignItems: 'center',
              width: '100%',
              height: '100%'
            }}
          >
            {elements[i * minor_axis + j]}
          </View>
        )
      }
      layout.push(
        <View
          style={{
            flex: 1,
            flexDirection: minorFlexDirection,
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%',
            height: '100%'
          }}
        >
          {row}
        </View>
      )
    }
    return (
      <View
        style={{
          flex: 1,
          flexDirection: majorFlexDirection,
          justifyContent: 'center',
          alignItems: 'center',
          width: '100%',
          height: '100%'
        }}
      >
        {layout}
      </View>
    )
  }, [])

  /**
   * Callback to get cartesian hold buttons
   */
  const getCartesianHoldButton = useCallback(
    (x, y, z, rx, ry, rz, text) => {
      return (
        <HoldButton
          rate_hz={10.0}
          holdCallback={() => {
            publishCartesianVelocity(
              LINEAR_SPEED * x,
              LINEAR_SPEED * y,
              LINEAR_SPEED * z,
              ANGULAR_SPEED * rx,
              ANGULAR_SPEED * ry,
              ANGULAR_SPEED * rz
            )
          }}
          cleanupCallback={() => {
            publishCartesianVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
          }}
          buttonStyle={buttonStyle}
        >
          {text}
        </HoldButton>
      )
    },
    [publishCartesianVelocity, buttonStyle, LINEAR_SPEED, ANGULAR_SPEED]
  )

  /**
   * Callback to get joint hold buttons
   */
  const getJointHoldButton = useCallback(
    (joint, velocity, text) => {
      return (
        <HoldButton
          rate_hz={10.0}
          holdCallback={() => {
            publishJointJog(joint, velocity)
          }}
          cleanupCallback={() => {
            publishCartesianVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
          }}
          buttonStyle={buttonStyle}
        >
          {text}
        </HoldButton>
      )
    },
    [publishJointJog, buttonStyle, publishCartesianVelocity]
  )

  /**
   * Callback to render the cartesian teleop controls.
   */
  const cartesianTeleop = useCallback(() => {
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
        {/* Translation trackpad */}
        <View
          style={{
            flex: 2,
            flexDirection: 'col',
            justifyContent: 'center',
            alignItems: 'center',
            width: '90%',
            height: '90%'
          }}
        >
          <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
            Position
          </p>
          {trackpadArrangement(
            getCartesianHoldButton(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'Left'),
            getCartesianHoldButton(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'Right'),
            getCartesianHoldButton(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'Up'),
            getCartesianHoldButton(0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 'Down'),
            getCartesianHoldButton(0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 'Forward'),
            getCartesianHoldButton(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 'Backward'),
            true
          )}
        </View>
        {/* Whitespace between trackpads */}
        <View
          style={{
            flex: 1,
            flexDirection: 'col',
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%',
            height: '100%'
          }}
        ></View>
        {/* Orientation trackpad */}
        <View
          style={{
            flex: 2,
            flexDirection: 'col',
            justifyContent: 'center',
            alignItems: 'center',
            width: '90%',
            height: '90%'
          }}
        >
          <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
            Orientation
          </p>
          {trackpadArrangement(
            getCartesianHoldButton(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 'Yaw +'),
            getCartesianHoldButton(0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 'Yaw -'),
            getCartesianHoldButton(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 'Pitch +'),
            getCartesianHoldButton(0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 'Pitch -'),
            getCartesianHoldButton(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 'Roll +'),
            getCartesianHoldButton(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 'Roll -'),
            true
          )}
        </View>
      </View>
    )
  }, [trackpadArrangement, getCartesianHoldButton, textFontSize, dimension])

  /**
   * Callback to render the joint teleop controls.
   */
  const jointTeleop = useCallback(() => {
    return arrayArrangement(
      6,
      2,
      [
        getJointHoldButton('j2n6s200_joint_1', 1.0, 'J1+'),
        getJointHoldButton('j2n6s200_joint_1', -1.0, 'J1-'),
        getJointHoldButton('j2n6s200_joint_2', 1.0, 'J2+'),
        getJointHoldButton('j2n6s200_joint_2', -1.0, 'J2-'),
        getJointHoldButton('j2n6s200_joint_3', 1.0, 'J3+'),
        getJointHoldButton('j2n6s200_joint_3', -1.0, 'J3-'),
        getJointHoldButton('j2n6s200_joint_4', 1.0, 'J4+'),
        getJointHoldButton('j2n6s200_joint_4', -1.0, 'J4-'),
        getJointHoldButton('j2n6s200_joint_5', 1.0, 'J5+'),
        getJointHoldButton('j2n6s200_joint_5', -1.0, 'J5-'),
        getJointHoldButton('j2n6s200_joint_6', 1.0, 'J6+'),
        getJointHoldButton('j2n6s200_joint_6', -1.0, 'J6-')
      ],
      !isPortrait
    )
  }, [arrayArrangement, getJointHoldButton, isPortrait])

  // Render the component
  return (
    <>
      <View
        style={{
          flex: 1,
          flexDirection: 'row',
          justifyContent: 'center',
          alignItems: 'center',
          width: '100%',
          height: '100%'
        }}
      >
        <View
          style={{
            flex: 1,
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%',
            height: '100%'
          }}
        >
          <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
            <input
              name='teleopMode'
              type='radio'
              checked={teleopMode === CARTESIAN_MODE}
              onChange={(e) => {
                if (e.target.checked) {
                  setTeleopMode(CARTESIAN_MODE)
                }
              }}
              style={{ transform: 'scale(2.0)', verticalAlign: 'middle', marginRight: '15px' }}
            />
            Move Fork
          </p>
        </View>
        <View
          style={{
            flex: 1,
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%',
            height: '100%'
          }}
        >
          <p className='transitionMessage' style={{ marginBottom: '0px', fontSize: textFontSize }}>
            <input
              name='teleopMode'
              type='radio'
              checked={teleopMode === JOINT_MODE}
              onChange={(e) => {
                if (e.target.checked) {
                  setTeleopMode(JOINT_MODE)
                }
              }}
              style={{ transform: 'scale(2.0)', verticalAlign: 'middle', marginRight: '15px' }}
            />
            Move Joints
          </p>
        </View>
      </View>
      <View
        style={{
          flex: 9,
          justifyContent: 'center',
          alignItems: 'center',
          width: '100%',
          height: '100%'
        }}
      >
        {teleopMode === CARTESIAN_MODE ? cartesianTeleop() : jointTeleop()}
      </View>
    </>
  )
}
TeleopSubcomponent.propTypes = {}
export default TeleopSubcomponent
