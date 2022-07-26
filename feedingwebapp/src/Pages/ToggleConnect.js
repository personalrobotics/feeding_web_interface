import React from 'react'
import { useROS } from 'react-ros'

function ToggleConnect() {
    var foo = useROS();
    console.log({...foo});

    const { isConnected, topics, url, changeUrl, toggleConnection } = useROS();

    console.log(topics);

    var new_topic = {
        path: "/fromRobot",
        msgType: "std_msgs/String",
        type: "topic"
    }

    topics[4] = new_topic

    return (
        <div>
            <p>
                <b>Simple connect:  </b><button onClick={toggleConnection}>Toggle Connect</button>  <br />
                <b>ROS url input:  </b><input name="urlInput" defaultValue={url} onChange={event => changeUrl(event.target.value)} />  <br />
                <b>ROS url to connect to:  </b> {url}  <br />
                <b>Status of ROS:</b> {isConnected ? "connected" : "not connected"}   <br />
                <b>Topics detected:</b><br />
                {topics.map((topic, i) => <li key={i}>    {topic.path}</li>)}
            </p>
        </div>
    );
}

export default ToggleConnect;