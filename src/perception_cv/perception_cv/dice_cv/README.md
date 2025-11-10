ros2 topic pub /dice_results custom_interface/msg/DiceResults "{dice: [
  {x: 100, y: 120, width: 60, height: 60, dice_number: 3, confidence: 0.95,
   pose: {position: {x: 0.1, y: 0.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  {x: 250, y: 120, width: 60, height: 60, dice_number: 5, confidence: 0.92,
   pose: {position: {x: 0.3, y: 0.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
]}"


Fake results for testing