import ROSLIB from 'roslib';

let ros = null;

/**
 * Get or create a persistent ROS connection.
 */
function getRos() {
    if (!ros) {
        ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

        ros.on('connection', () => console.log('[ROS2] ✅ Connected to rosbridge WebSocket'));
        ros.on('error', (err) => console.error('[ROS2] ❌ Connection error:', err));
        ros.on('close', () => console.warn('[ROS2] ⚠️ Connection closed'));
    }
    return ros;
}

/* -------------------------------------------------------------------------- */
/* 🧍 PLAYER DATA                                                             */
/* -------------------------------------------------------------------------- */

/**
 * Subscribe to the /players topic.
 * @param {Function} callback - Called with msg.players array (from Players.msg)
 * @returns {Function} unsubscribe function
 */
export function subscribeToPlayers(callback) {
    const ros = getRos();

    const topic = new ROSLIB.Topic({
        ros,
        name: '/players',
        messageType: 'custom_interface/msg/Players',
    });

    topic.subscribe((msg) => {
        // msg.players is the array from Players.msg
        callback(msg.players);
    });

    return () => topic.unsubscribe();
}

/* -------------------------------------------------------------------------- */
/* 🎮 GAME CONTROL                                                            */
/* -------------------------------------------------------------------------- */

/**
 * Call the /start_round service to begin a round.
 * @param {Function} callback - Called with service response
 */
export function callStartRoundService(callback) {
    const ros = getRos();

    const service = new ROSLIB.Service({
        ros,
        name: '/start_round',
        serviceType: 'custom_interface/srv/StartRound',
    });

    const request = new ROSLIB.ServiceRequest({ start: true });

    service.callService(request, (result) => {
        console.log('[ROS2] ▶️ Round start response:', result);
        if (callback) callback(result);
    });
}

/**
 * Subscribe to /round_result to receive round completion and outcome.
 * @param {Function} callback - Called with RoundResult.msg
 * @returns {Function} unsubscribe function
 */
export function subscribeToRoundResult(callback) {
    const ros = getRos();

    const topic = new ROSLIB.Topic({
        ros,
        name: '/round_result',
        messageType: 'custom_interface/msg/RoundResult',
    });

    topic.subscribe((msg) => {
        console.log('[ROS2] 🎯 Received round result:', msg);
        callback(msg);
    });

    return () => topic.unsubscribe();
}

/**
 * Subscribe to /dice_results to receive dice roll outcomes.
 * @param {Function} callback - Called with DiceResults.msg 
 * @returns  {Function} unsubscribe function
 */
export function subscribeToDiceResults(callback) {
  const ros = getRos();
  const topic = new ROSLIB.Topic({
    ros,
    name: '/dice_results', // adjust if your topic name differs
    messageType: 'custom_interface/msg/DiceResults',
  });
  topic.subscribe((msg) => callback && callback(msg));
  return () => topic.unsubscribe();
}