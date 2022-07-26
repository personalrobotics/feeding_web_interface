import React from 'react';
import { ROS } from 'react-ros';
import EchoTopic from './EchoTopic';
import ToggleConnect from './ToggleConnect';
import Home from './Home/Home.js'

const RosSubs = () => {
    return (
        <ROS>
            <ToggleConnect/>
            <EchoTopic />
            {/* <Home /> */}
        </ROS>
    );

}

export default RosSubs;