import { MEAL_STATE } from './GlobalState'
// The RealSense's default video stream is 640x480
export const REALSENSE_WIDTH = 640
export const REALSENSE_HEIGHT = 480
export const state_dict = {
  R_MovingAbovePlate: ['Pre Meal', 'Bite Selection', MEAL_STATE.U_PreMeal],
  R_BiteAcquisition: ['Bite Selection', 'Bite Acquisition Check', MEAL_STATE.U_BiteSelection],
  R_MovingToStagingLocation: ['Bite Acquisition Check', 'Bite Initiation', MEAL_STATE.U_BiteAcquisitionCheck],
  R_MovingToMouth: ['Bite Initiation', 'Bite Done', MEAL_STATE.U_BiteInitiation],
  R_StowingArm: ['Bite Done', 'Post Meal', MEAL_STATE.U_BiteDone]
}
