// React imports
import React from 'react'

// Local imports
import { ROBOT_COMPRESSED_IMG_TOPICS } from '../Pages/Constants'
import VideoStream from './VideoStream'

function RobotVideoStreams() {
    console.log('Rendering RobotVideoStreams', ROBOT_COMPRESSED_IMG_TOPICS)
    return (
        <>
        {ROBOT_COMPRESSED_IMG_TOPICS.map((topic, i) => (
            <VideoStream topic={topic} key={i} />
        ))}
        </>
    )
}

export default RobotVideoStreams
