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
  R_MovingAbovePlate: ['Pre Meal', 'Bite Selection', MEAL_STATE.U_PreMeal],
  R_BiteAcquisition: ['Bite Selection', 'Bite Acquisition Check', MEAL_STATE.U_BiteSelection],
  R_MovingToStagingLocation: ['Bite Acquisition Check', 'Bite Initiation', MEAL_STATE.U_BiteAcquisitionCheck],
  R_MovingToMouth: ['Bite Initiation', 'Bite Done', MEAL_STATE.U_BiteInitiation],
  R_StowingArm: ['Bite Done', 'Post Meal', MEAL_STATE.U_BiteDone]
}
