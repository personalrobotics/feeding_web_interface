import React from 'react';
import { ROS } from 'react-ros';
import EchoTopic from './EchoTopic';
import ToggleConnect from './ToggleConnect';

const RosSubs = () => {
    return (
        <ROS>
            <ToggleConnect/>
            <EchoTopic />
        </ROS>
    );

}

export default RosSubs;