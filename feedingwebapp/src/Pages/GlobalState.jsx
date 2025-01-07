/**
 * zustand is a state management system for React apps. It lets us manage global
 * state, i.e., state that persists beyond individual pages of the app. In
 * practice, it stores this state in cookies.
 */
import create from 'zustand'
import { persist } from 'zustand/middleware'

/**
 * APP_PAGE specifies what page the app is on. It can take the following values:
 *  - Settings: the settings page, where the user can configure parameters of
 *    that dictate how the robot will behave.
 *  - Home: the main screen, where the robot will display its state, solicit
 *    user input, etc.
 */
export const APP_PAGE = {
  Settings: 'settings',
  Home: 'home'
}

/**
 * MEAL_STATE stores the current state of the meal, and configures the
 * functionality of APP_PAGE.Home. States pre-pended with "U_" are waiting
 * for user input whereas states pre-pended with "R_" are waiting for the
 * robot to finish moving. The states are:
 *   - U_PreMeal: Waiting for the user to click "Start Feeding."
 *   - U_LabelGeneration: Waiting for the user to label the food items on the
 *     plate.
 *   - U_UnderstandPlate: Waiting for the robot to generate a visually descriptive
 *     caption that describes the food items on the plate.
 *   - U_DetectingFoods: Waiting for the robot to detect the food items on the
 *     plate.
 *   - R_MovingAbovePlate: Waiting for the robot to move above the plate.
 *   - U_BiteSelection: Waiting for the user to select the food item they want.
 *   - R_BiteAcquisition: Waiting for the robot to execute one bite acquisition
 *     attempt.
 *   - R_MovingToRestingPosition: Waiting for the robot to move to resting
 *     position.
 *   - U_BiteAcquisitionCheck: Waiting for the user to specify whether the
 *     bite acquisition was succesful or not.
 *   - R_MovingToStagingConfiguration: Waiting for the robot to move to the
 *     staging configuration.
 *   - R_DetectingFace: Waiting for the robot to detect a face.
 *   - R_MovingToMouth: Waiting for the robot to finish moving to the user's
 *     mouth.
 *   - R_MovingFromMouth: Waiting for the robot to move
 *     from the user's mouth to the staging configuration. This is a separate
 *     action from R_MovingToStagingConfiguration since it is cartesian.
 *   - U_BiteDone: Waiting for the user to indicate that they are done eating
 *     the bite.
 *   - R_StowingArm: Waiting for the robot to stow the arm.
 *   - U_PostMeal: Waiting on the user to start another meal.
 */
export const MEAL_STATE = {
  U_PreMeal: 'U_PreMeal',
  U_LabelGeneration: 'U_LabelGeneration',
  U_UnderstandPlate: 'U_UnderstandPlate',
  U_DetectingFoods: 'U_DetectingFoods',
  R_MovingAbovePlate: 'R_MovingAbovePlate',
  U_BiteSelection: 'U_BiteSelection',
  R_BiteAcquisition: 'R_BiteAcquisition',
  R_MovingToRestingPosition: 'R_MovingToRestingPosition',
  U_BiteAcquisitionCheck: 'U_BiteAcquisitionCheck',
  R_MovingToStagingConfiguration: 'R_MovingToStagingConfiguration',
  R_DetectingFace: 'R_DetectingFace',
  R_MovingToMouth: 'R_MovingToMouth',
  R_MovingFromMouth: 'R_MovingFromMouth',
  U_BiteDone: 'U_BiteDone',
  R_StowingArm: 'R_StowingArm',
  U_PostMeal: 'U_PostMeal'
}

/**
 * A set containing the states where the robot does not move.
 */
let NON_MOVING_STATES = new Set()
NON_MOVING_STATES.add(MEAL_STATE.U_PreMeal)
NON_MOVING_STATES.add(MEAL_STATE.U_LabelGeneration)
NON_MOVING_STATES.add(MEAL_STATE.U_UnderstandPlate)
NON_MOVING_STATES.add(MEAL_STATE.U_DetectingFoods)
NON_MOVING_STATES.add(MEAL_STATE.U_BiteSelection)
NON_MOVING_STATES.add(MEAL_STATE.U_BiteAcquisitionCheck)
NON_MOVING_STATES.add(MEAL_STATE.R_DetectingFace)
NON_MOVING_STATES.add(MEAL_STATE.U_BiteDone)
NON_MOVING_STATES.add(MEAL_STATE.U_PostMeal)
export { NON_MOVING_STATES }

/**
 * SETTINGS_STATE controls which settings page to display.
 *  - MAIN: The main page, with options to navigate to the other pages.
 *  - BITE_TRANSFER: Allow the user to customize how close the robot gets
 *    to their mouth and the speed of approach/departure.
 *  - ABOVE_PLATE: Allow the user to customize how high the fixed above plate
 *    arm configuration.
 *  - RESTING_CONFIGURATION: Allow the user to customize the fixed resting
 *    arm configuration.
 *  - STAGING_CONFIGURATION: Allow the user to customize the fixed staging
 *    arm configuration.
 *  - STOW_CONFIGURATION: Allow the user to customize the fixed stow arm
 *    configuration.
 */
export const SETTINGS_STATE = {
  MAIN: 'MAIN',
  BITE_TRANSFER: 'BITE_TRANSFER',
  ABOVE_PLATE: 'ABOVE_PLATE',
  RESTING_CONFIGURATION: 'RESTING_CONFIGURATION',
  STAGING_CONFIGURATION: 'STAGING_CONFIGURATION',
  STOW_CONFIGURATION: 'STOW_CONFIGURATION',
  PLANNING_SCENE: 'PLANNING_SCENE',
  AUTO_CONTINUE: 'AUTO_CONTINUE',
  SEMANTIC_LABELING: 'SEMANTIC_LABELING'
}

// The name of the default parameter namespace
export const DEFAULT_NAMESPACE = 'default'

/**
 * useGlobalState is a hook to store and manipulate web app state that we want
 * to persist across re-renders and refreshes. It won't persist if cookies are
 * cleared.
 */
export const useGlobalState = create(
  persist(
    (set, get) => ({
      // The current app page
      appPage: APP_PAGE.Home,
      // The app's current meal state
      mealState: MEAL_STATE.U_PreMeal,
      // The app's previous meal state
      prevMealState: null,
      // Whether the app is currently in a non-moving state (i.e., the robot will
      // not move unless the user initiates it)
      inNonMovingState: true,
      // The timestamp when the robot transitioned to its current meal state
      mealStateTransitionTime: Date.now(),
      // The currently displayed settings page
      settingsState: SETTINGS_STATE.MAIN,
      settingsPresets: { current: DEFAULT_NAMESPACE, customNames: [] },
      // The goal for the bite acquisition action, including the most recent
      // food item that the user selected in "bite selection"
      biteAcquisitionActionGoal: null,
      // The goal for the move to mouth action, including the most recent
      // message received from the face detection node where a
      // face was detected and within the distance bounds of the camera.
      moveToMouthActionGoal: null,
      // Last RobotMotion action feedback message
      lastMotionActionFeedback: null,
      // Whether or not the currently-executing robot motion was paused by the user.
      // NOTE: `paused` may no longer need to be in global state now that we have
      // the `inNonMovingState` flag.
      paused: false,
      // Store the user;s current settings for teleop speeds
      teleopLinearSpeed: 0.1, // m/s
      teleopAngularSpeed: 0.15, // rad/s
      teleopJointSpeed: 0.2, // rad/s
      // Flag to indicate whether to auto-continue after face detection
      faceDetectionAutoContinue: false,
      // Flag to indicate whether to auto-continue in bite done after food-on-fork detection
      biteDoneAutoContinue: false,
      biteDoneAutoContinueSecs: 3.0,
      biteDoneAutoContinueProbThresh: 0.25,
      // Flags to indicate whether to auto-continue in bite acquisition check based on food-on-fork
      // detection
      biteAcquisitionCheckAutoContinue: false,
      biteAcquisitionCheckAutoContinueSecs: 3.0,
      biteAcquisitionCheckAutoContinueProbThreshLower: 0.25,
      biteAcquisitionCheckAutoContinueProbThreshUpper: 0.75,
      // Whether any of the settings pages is currently at the user's mouth
      // or not. This is in the off-chance that the mealState is not at the user's
      // mouth, the settings page is, and the user refreshes -- the page should
      // call MoveFromMouth instead of just MoveToStaging.
      settingsPageAtMouth: false,
      // The button the user most recently clicked on the BiteDone page. In practice,
      // this is the state we transition to after R_MovingFromMouth. In practice,
      // it is either R_MovingAbovePlate, R_MovingToRestingPosition, or R_DetectingFace.
      mostRecentBiteDoneResponse: MEAL_STATE.R_DetectingFace,
      // How much the video on the Bite Selection page should be zoomed in.
      biteSelectionZoom: 1.0,
      // Flag to indicate whether the user has visited the Label Generation page
      // and clicked the "Begin Meal" button to confirm the labels
      labelGenerationConfirmed: false,
      // A set of labels inputted by the user defining the food items on the plate
      foodItemLabels: new Set([]),
      // A visually descriptive caption generated by GPT-4o for the food items on the plate
      gpt4oCaption: '',
      // Flag to indicate whether to use the semantic labels user interface or the default user
      // interface
      semanticLabeling: true,
      // Setters for global state
      setAppPage: (appPage) =>
        set(() => ({
          appPage: appPage,
          settingsState: SETTINGS_STATE.MAIN,
          // Sometimes the settings menu leaves the robot in a paused state.
          // Thus, we reset it to an unpaused state.
          paused: false
        })),
      setMealState: (mealState, mostRecentBiteDoneResponse = null) =>
        set(() => {
          let prevMealState = get().mealState
          console.log('Setting meal state to', mealState, 'from', prevMealState)
          let retval = {
            mealState: mealState,
            mealStateTransitionTime: Date.now(),
            settingsPageAtMouth: false // Reset this flag when the meal state changes
          }
          // Only update the previous state if it is not a self-transition (to
          // account for cases where a MoveTo action result message is reveived twice)
          if (prevMealState !== mealState) {
            retval.prevMealState = prevMealState
          }
          if (mostRecentBiteDoneResponse) {
            retval.mostRecentBiteDoneResponse = mostRecentBiteDoneResponse
          }
          if (NON_MOVING_STATES.has(mealState)) {
            retval.inNonMovingState = true
            console.log('Setting inNonMovingState to true through setMealState')
          } else {
            retval.inNonMovingState = false
            console.log('Setting inNonMovingState to false through setMealState')
          }
          return retval
        }),
      setInNonMovingState: (inNonMovingState) =>
        set(() => {
          console.log('Setting inNonMovingState to', inNonMovingState, 'through setInNonMovingState')
          return {
            inNonMovingState: inNonMovingState
          }
        }),
      setSettingsState: (settingsState) =>
        set(() => ({
          settingsState: settingsState
        })),
      setSettingsPresets: (settingsPresets) =>
        set(() => ({
          settingsPresets: settingsPresets
        })),
      setBiteAcquisitionActionGoal: (biteAcquisitionActionGoal) =>
        set(() => ({
          biteAcquisitionActionGoal: biteAcquisitionActionGoal
        })),
      setLastMotionActionFeedback: (lastMotionActionFeedback) =>
        set(() => ({
          lastMotionActionFeedback: lastMotionActionFeedback
        })),
      setMoveToMouthActionGoal: (moveToMouthActionGoal) =>
        set(() => {
          console.log('setMoveToMouthActionGoal called with', moveToMouthActionGoal)
          return {
            moveToMouthActionGoal: moveToMouthActionGoal
          }
        }),
      setPaused: (paused) =>
        set(() => {
          let retval = { paused: paused }
          if (paused) {
            // If the robot is paused, we should store this as a non-moving state
            retval.inNonMovingState = true
          } else {
            // If the robot is unpaused, we should check if the meal state is moving
            if (!NON_MOVING_STATES.has(get().mealState)) {
              retval.inNonMovingState = false
              console.log('Setting inNonMovingState to false through setPaused')
            } else {
              retval.inNonMovingState = true
              console.log('Setting inNonMovingState to true through setPaused')
            }
          }
          return retval
        }),
      setTeleopLinearSpeed: (teleopLinearSpeed) =>
        set(() => ({
          teleopLinearSpeed: teleopLinearSpeed
        })),
      setTeleopAngularSpeed: (teleopAngularSpeed) =>
        set(() => ({
          teleopAngularSpeed: teleopAngularSpeed
        })),
      setTeleopJointSpeed: (teleopJointSpeed) =>
        set(() => ({
          teleopJointSpeed: teleopJointSpeed
        })),
      setFaceDetectionAutoContinue: (faceDetectionAutoContinue) =>
        set(() => ({
          faceDetectionAutoContinue: faceDetectionAutoContinue
        })),
      setBiteDoneAutoContinue: (biteDoneAutoContinue) =>
        set(() => ({
          biteDoneAutoContinue: biteDoneAutoContinue
        })),
      setBiteDoneAutoContinueSecs: (biteDoneAutoContinueSecs) =>
        set(() => ({
          biteDoneAutoContinueSecs: biteDoneAutoContinueSecs
        })),
      setBiteDoneAutoContinueProbThresh: (biteDoneAutoContinueProbThresh) =>
        set(() => ({
          biteDoneAutoContinueProbThresh: biteDoneAutoContinueProbThresh
        })),
      setBiteAcquisitionCheckAutoContinue: (biteAcquisitionCheckAutoContinue) =>
        set(() => ({
          biteAcquisitionCheckAutoContinue: biteAcquisitionCheckAutoContinue
        })),
      setBiteAcquisitionCheckAutoContinueSecs: (biteAcquisitionCheckAutoContinueSecs) =>
        set(() => ({
          biteAcquisitionCheckAutoContinueSecs: biteAcquisitionCheckAutoContinueSecs
        })),
      setBiteAcquisitionCheckAutoContinueProbThreshLower: (biteAcquisitionCheckAutoContinueProbThreshLower) =>
        set(() => ({
          biteAcquisitionCheckAutoContinueProbThreshLower: biteAcquisitionCheckAutoContinueProbThreshLower
        })),
      setBiteAcquisitionCheckAutoContinueProbThreshUpper: (biteAcquisitionCheckAutoContinueProbThreshUpper) =>
        set(() => ({
          biteAcquisitionCheckAutoContinueProbThreshUpper: biteAcquisitionCheckAutoContinueProbThreshUpper
        })),
      setSettingsPageAtMouth: (settingsPageAtMouth) =>
        set(() => ({
          settingsPageAtMouth: settingsPageAtMouth
        })),
      setBiteSelectionZoom: (biteSelectionZoom) =>
        set(() => ({
          biteSelectionZoom: biteSelectionZoom
        })),
      setLabelGenerationConfirmed: (labelGenerationConfirmed) =>
        set(() => ({
          labelGenerationConfirmed: labelGenerationConfirmed
        })),
      setFoodItemLabels: (foodItemLabels) =>
        set(() => ({
          foodItemLabels: foodItemLabels
        })),
      setGPT4oCaption: (gpt4oCaption) =>
        set(() => ({
          gpt4oCaption: gpt4oCaption
        })),
      setSemanticLabeling: (semanticLabeling) =>
        set(() => ({
          semanticLabeling: semanticLabeling
        }))
    }),
    { name: 'ada_web_app_global_state' }
  )
)
