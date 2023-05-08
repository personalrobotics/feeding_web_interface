import { MEAL_STATE } from './GlobalState'
// The RealSense's default video stream is 640x480
export const REALSENSE_WIDTH = 640
export const REALSENSE_HEIGHT = 480
/**
 * A dictionary containing images for icons of the meal states where the robot moves.
 * Those meal states are used as keys.
 * Each key is paired with a value of an svg icon image of that meal state.
 * TODO: update the dictionary with correct meal state and image pairings
 * for the Back button of the Footer.
 */
let robot_moving_state_icon_image_dict = {}
robot_moving_state_icon_image_dict[MEAL_STATE.R_MovingAbovePlate] = '/robot_state_imgs/move_above_plate_position.svg'
robot_moving_state_icon_image_dict[MEAL_STATE.R_BiteAcquisition] = '/robot_state_imgs/move_to_bite_acquisition_position.svg'
robot_moving_state_icon_image_dict[MEAL_STATE.R_MovingToStagingLocation] = '/robot_state_imgs/move_to_staging_position_footer.svg'
robot_moving_state_icon_image_dict[MEAL_STATE.R_MovingToMouth] = '/robot_state_imgs/move_to_mouth_position.svg'
export { robot_moving_state_icon_image_dict }
