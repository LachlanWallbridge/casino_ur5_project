// frontend/src/ui_bridge.js
import ROSLIB from 'roslib';

let ros = null;

export function subscribeToPlayers(callback) {
    if (!ros) {
        ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        ros.on('connection', () => console.log('Connected to ROS2 WebSocket'));
        ros.on('error', (err) => console.error('ROS2 connection error', err));
        ros.on('close', () => console.log('ROS2 connection closed'));
    }

    const listener = new ROSLIB.Topic({
        ros,
        name: '/players',       // ROS2 topic for Players.msg
        messageType: 'custom_interface/msg/Players'
    });

    listener.subscribe((msg) => {
        // msg.players is the array from Players.msg
        callback(msg.players);
    });

    return () => listener.unsubscribe(); // cleanup
}
