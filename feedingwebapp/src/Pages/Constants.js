// The RealSense's default video stream is 640x480
import { MEAL_STATE } from './GlobalState'
//import MovingAbovePlate from './MealStates/MovingAbovePlate'

export const REALSENSE_WIDTH = 640
export const REALSENSE_HEIGHT = 480

export const move_above_plate_dict = {
  prev_text: 'Pre Meal',
  next_text: 'Bite Selection',
  prev_state: MEAL_STATE.U_PreMeal
}

export const bite_acquistion_dict = {
  prev_text: 'Bite Selection',
  next_text: 'Bite Acquisition Check',
  prev_state: MEAL_STATE.U_BiteSelection
}

export const move_to_staging_dict = {
  prev_text: 'Bite Acquisition Check',
  next_text: 'Bite Initiation',
  prev_state: MEAL_STATE.U_BiteAcquisitionCheck
}

export const move_to_mouth_dict = {
  prev_text: 'Bite Initiation',
  next_text: 'Bite Done',
  prev_state: MEAL_STATE.U_BiteInitiation
}

export const stow_arm_dict = {
  prev_text: 'Bite Done',
  next_text: 'Post Meal',
  prev_state: MEAL_STATE.U_BiteDone
}
