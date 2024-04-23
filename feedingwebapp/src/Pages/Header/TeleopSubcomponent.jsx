// React Imports
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useId, Label, SpinButton } from '@fluentui/react-components'
import { useMediaQuery } from 'react-responsive'
import { View } from 'react-native'

// Local Imports
import { useROS, createROSTopic, createROSMessage, createROSActionClient, callROSAction, destroyActionClient } from '../../ros/ros_helpers'
import '../Home/Home.css'
import {
  SERVO_CARTESIAN_TOPIC,
  SERVO_CARTESIAN_TOPIC_MSG,
  SERVO_JOINT_TOPIC,
  SERVO_JOINT_TOPIC_MSG,
  START_SERVO_ACTION_NAME,
  START_SERVO_ACTION_TYPE,
  STOP_SERVO_ACTION_NAME,
  STOP_SERVO_ACTION_TYPE
} from '../Constants'
import { useGlobalState } from '../GlobalState'
import HoldButton from '../../buttons/HoldButton'

/**
 * The TeleopSubcomponent renders controls for the user to teleoperate the robot,
 * either in cartesian or joint space.
 */
const TeleopSubcomponent = () => {
  // Get local state
  const CARTESIAN_LINEAR_MODE = 'cartesian_linear'
  const CARTESIAN_ANGULAR_MODE = 'cartesian_angular'
  const JOINT_MODE = 'joint'
  const [teleopMode, setTeleopMode] = useState(CARTESIAN_LINEAR_MODE)
  const speedSpinButtonID = useId()

  // Cartesian teleop speeds
  const LINEAR_MAX_SPEED = useMemo(() => 0.3, []) // m/s
  const LINEAR_MIN_SPEED = useMemo(() => 0.05, []) // m/s
  const teleopLinearSpeed = useGlobalState((state) => state.teleopLinearSpeed)
  const setTeleopLinearSpeed = useGlobalState((state) => state.setTeleopLinearSpeed)
  const ANGULAR_MAX_SPEED = useMemo(() => 0.6, []) // rad/s
  const ANGULAR_MIN_SPEED = useMemo(() => 0.3, []) // rad/s
  const teleopAngularSpeed = useGlobalState((state) => state.teleopAngularSpeed)
  const setTeleopAngularSpeed = useGlobalState((state) => state.setTeleopAngularSpeed)
  const JOINT_MAX_SPEED = useMemo(() => 0.6, []) // rad/s
  const JOINT_MIN_SPEED = useMemo(() => 0.2, []) // rad/s
  const teleopJointSpeed = useGlobalState((state) => state.teleopJointSpeed)
  const setTeleopJointSpeed = useGlobalState((state) => state.setTeleopJointSpeed)

  // Style configuration
  const isPortrait = useMediaQuery({ query: '(orientation: portrait)' })
  let textFontSize = isPortrait ? '2.2vh' : '2.5vw'
  const buttonStyle = useMemo(() => {
    return {
      width: '100%',
      height: '100%',
      border: '1px solid black',
      padding: '0rem',
      borderRadius: '0rem'
    }
  }, [])

  // Callback for when the user changes the speed
  const onSpeedChange = useCallback(
    (_ev, data) => {
      let value = data.value ? data.value : parseFloat(data.displayValue)
      let min_val =
        teleopMode === CARTESIAN_LINEAR_MODE
          ? LINEAR_MIN_SPEED
          : teleopMode === CARTESIAN_ANGULAR_MODE
          ? ANGULAR_MIN_SPEED
          : JOINT_MIN_SPEED
      let max_val =
        teleopMode === CARTESIAN_LINEAR_MODE
          ? LINEAR_MAX_SPEED
          : teleopMode === CARTESIAN_ANGULAR_MODE
          ? ANGULAR_MAX_SPEED
          : JOINT_MAX_SPEED
      if (value < min_val) {
        value = min_val
      }
      if (value > max_val) {
        value = max_val
      }
      let setter =
        teleopMode === CARTESIAN_LINEAR_MODE
          ? setTeleopLinearSpeed
          : teleopMode === CARTESIAN_ANGULAR_MODE
          ? setTeleopAngularSpeed
          : setTeleopJointSpeed
      setter(value)
    },
    [
      teleopMode,
      setTeleopLinearSpeed,
      setTeleopAngularSpeed,
      setTeleopJointSpeed,
      LINEAR_MIN_SPEED,
      LINEAR_MAX_SPEED,
      ANGULAR_MIN_SPEED,
      ANGULAR_MAX_SPEED,
      JOINT_MIN_SPEED,
      JOINT_MAX_SPEED
    ]
  )

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
        velocities: [teleopJointSpeed * velocity]
      })
      jointTopic.publish(msg)
    },
    [jointTopic, teleopJointSpeed]
  )

  /**
   * Arrange the specified 6 elements in a trackpad-like arrangement.
   *
   * e11 and e12 are the "first-level," opposite from one another.
   * e21, and e22 are the "second-level," opposite from one another.
   * e31 and e32 are the "third-level," opposite from one another.
   */
  const trackpadArrangement = useCallback((e11, e12, e21, e22, e31, e32, horizontal = true) => {
    let mainFlexDirection = horizontal ? 'row' : 'column'
    let subFlexDirection = horizontal ? 'column' : 'row'

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
    let majorFlexDirection = horizontal ? 'row' : 'column'
    let minorFlexDirection = horizontal ? 'column' : 'row'

    let layout = []
    for (let i = 0; i < major_axis; i++) {
      let row = []
      for (let j = 0; j < minor_axis; j++) {
        row.push(
          <View
            key={i.toString() + '-' + j.toString()}
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
          key={i.toString()}
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
            console.log('Cartesian hold callback called')
            publishCartesianVelocity(
              teleopLinearSpeed * x,
              teleopLinearSpeed * y,
              teleopLinearSpeed * z,
              teleopAngularSpeed * rx,
              teleopAngularSpeed * ry,
              teleopAngularSpeed * rz
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
    [publishCartesianVelocity, buttonStyle, teleopLinearSpeed, teleopAngularSpeed]
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
  const cartesianLinearTeleop = useCallback(() => {
    return trackpadArrangement(
      getCartesianHoldButton(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'Up'),
      getCartesianHoldButton(0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 'Down'),
      getCartesianHoldButton(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'Left'),
      getCartesianHoldButton(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'Right'),
      getCartesianHoldButton(0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 'Forward'),
      getCartesianHoldButton(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 'Backward'),
      false
    )
  }, [trackpadArrangement, getCartesianHoldButton])

  /**
   * Callback to render the cartesian teleop controls.
   */
  const cartesianAngularTeleop = useCallback(() => {
    return trackpadArrangement(
      getCartesianHoldButton(0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 'Pitch +'),
      getCartesianHoldButton(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 'Pitch -'),
      getCartesianHoldButton(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 'Yaw +'),
      getCartesianHoldButton(0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 'Yaw -'),
      getCartesianHoldButton(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 'Roll +'),
      getCartesianHoldButton(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 'Roll -'),
      false
    )
  }, [trackpadArrangement, getCartesianHoldButton])

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
    <View
      style={{
        flex: 9,
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
              checked={teleopMode === CARTESIAN_LINEAR_MODE}
              onChange={(e) => {
                if (e.target.checked) {
                  setTeleopMode(CARTESIAN_LINEAR_MODE)
                }
              }}
              style={{ transform: 'scale(2.0)', verticalAlign: 'middle', marginRight: '15px' }}
            />
            Move
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
              checked={teleopMode === CARTESIAN_ANGULAR_MODE}
              onChange={(e) => {
                if (e.target.checked) {
                  setTeleopMode(CARTESIAN_ANGULAR_MODE)
                }
              }}
              style={{ transform: 'scale(2.0)', verticalAlign: 'middle', marginRight: '15px' }}
            />
            Rotate
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
            Joints
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
        {/* Allow users to tune to speed of the current teleop mode */}
        <View
          style={{
            flex: 1,
            flexDirection: 'row',
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%'
          }}
        >
          <Label
            htmlFor={speedSpinButtonID}
            style={{
              fontSize: textFontSize,
              width: '90%',
              color: 'black',
              textAlign: 'center'
            }}
          >
            Speed ({teleopMode === CARTESIAN_LINEAR_MODE ? 'm/s' : 'rad/s'})
          </Label>
          <SpinButton
            value={
              teleopMode === CARTESIAN_LINEAR_MODE
                ? teleopLinearSpeed
                : teleopMode === CARTESIAN_ANGULAR_MODE
                ? teleopAngularSpeed
                : teleopJointSpeed
            }
            id={speedSpinButtonID}
            step={0.05}
            onChange={onSpeedChange}
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
        {/* Render the controls for the mode */}
        <View
          style={{
            flex: 5,
            flexDirection: 'column',
            justifyContent: 'center',
            alignItems: 'center',
            width: '100%'
          }}
        >
          {teleopMode === CARTESIAN_LINEAR_MODE
            ? cartesianLinearTeleop()
            : teleopMode === CARTESIAN_ANGULAR_MODE
            ? cartesianAngularTeleop()
            : jointTeleop()}
        </View>
      </View>
    </View>
  )
}
TeleopSubcomponent.propTypes = {}
export default TeleopSubcomponent