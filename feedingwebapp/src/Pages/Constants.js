import { MEAL_STATE } from './GlobalState'
// The RealSense's default video stream is 640x480
export const REALSENSE_WIDTH = 640
export const REALSENSE_HEIGHT = 480
/**
 * A dictionary containing information for the meal states where the robot moves.
 * The meal states are used as keys.
 * Each key is paired with a value of an array that has 3 elements in it.
 * First 2 array elements contain the text used in pause modal for previous and next meal state of current meal state.
 * The 3rd array element contains the previous meal state of current meal state.
 */
export const pause_modal_state_info_dict = {
  R_MovingAbovePlate: ['stowing the arm', 'moving above plate', MEAL_STATE.R_StowingArm],
  R_BiteAcquisition: ['moving above plate', 'acquiring bite', MEAL_STATE.R_MovingAbovePlate],
  R_MovingToStagingLocation: ['moving above plate', 'moving to ready position', MEAL_STATE.R_MovingAbovePlate],
  R_MovingToMouth: ['moving to ready position', 'moving to mouth', MEAL_STATE.R_MovingToStagingLocation],
  R_StowingArm: ['moving above plate', 'stowing the arm', MEAL_STATE.R_MovingAbovePlate]
}
