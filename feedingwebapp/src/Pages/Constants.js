/**
 * State 1: "Moving above the plate": The Robot arm is physically moving towards the plate. 
 
 * State 2: "Feeding": During this state, the Robot UI displays the various food items it is detecting on the plate, allowing the user to select one item. During this state, the Robot arm is stationary on top of the food plate. 

 * State 3: "Acquiring Food Item": During this state, the Robot is physically moving towards the selected foot item on the plate. (Generally, this would start with a vertical motion towards the food item on the plate followed by a motion kind of closer to the user's face but not directly in front of the mouth.)

 * State 4: "Waiting for user to open mouth": During this state, the robot arm is stationary. It is waiting for an input from the user to convey that they have opened their mouth and are ready for the bite. (Generally, this can either be indicated by a button click or a detection of an open mouth)

 * State 5: "Moving closer to your mouth": During this state, the robot arm is physically moving closer to the user's mouth. 

 * State 6: "Waiting for user to complete the bite": This is when the robot is stationary waiting for the user to take the bite from the fork, chew, and finish the bite. 

 * State 7: "Not Eating": During this state, the robot should be stationary. Usually starts off with this state. 
 * 
 * State 8: "Emergency Termination": During this state, the robot stops all its motions and stays in the position it was present at.
 */
export const States = {
    1: "Moving_above_the_plate",
    2: "Feeding",
    3: "Acquiring_Food_Item",
    4: "Waiting_for_user_to_open_mouth",
    5: "Moving_Closer_to_your_Mouth",
    6: "Waiting_for_user_to_complete_bite",
    7: "Not_Eating",
    8: "Emergency_Termination"
}