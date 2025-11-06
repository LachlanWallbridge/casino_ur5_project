// frontend/src/ui_bridge.js
// Stub ROS2 bridge; replace with real ROS2 subscription later
export function subscribeToPlayers(callback) {
    // Example: simulate Players.msg coming in
    let playersMsg = [
        { player_id: "100", position: 1 },
        { player_id: "151", position: 2 },
        { player_id: "200", position: 3 }
    ];

    const interval = setInterval(() => {
        // You could shuffle positions or simulate joins/leaves
        callback(playersMsg);
    }, 2000);

    return () => clearInterval(interval); // unsubscribe
}
