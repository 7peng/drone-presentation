// Scenario for the live demo. World units are meters, Y is up, home is the origin.
// Tweak freely; the sim is deterministic for a given seed, so rehearsals match the real thing.
export const SCENARIO = {
  seed: 7,
  cruiseAlt: 40,         // m above home
  cruiseSpeed: 14,       // m/s horizontal
  climbRate: 5,          // m/s
  landRate: 3,           // m/s
  accel: 6,              // m/s² velocity-change limit
  waypointRadius: 5,     // m

  // Mission waypoints [x, z] flown at cruise altitude, then back home.
  waypoints: [[70, -120], [190, -210], [320, -150], [360, 10]],

  // Each drone gets its own pad; offset keeps them from overlapping.
  ours: { home: [0, 0] },
  baseline: { home: [10, 6] },

  jammer: {
    position: [260, -260],   // tower location [x, z]
    radius: 900,             // dome expands to cover the whole map
    autoAtWaypoint: 2,       // auto-jam when ours reaches this waypoint index (null = only manual J)
  },

  // Navigation error models after GPS loss (see sim.js).
  nav: {
    ours: { pull: 0.7, noise: 1.1 },               // learned estimator: bounded, mean-reverting error
    baseline: { bias: 0.14, noise: 0.25 },         // IMU dead reckoning: error grows ~ t²
  },
};
