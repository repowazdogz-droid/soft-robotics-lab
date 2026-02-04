import type { Challenge } from '../types';

export const CHALLENGES: Challenge[] = [
  {
    id: 'ball-in-basket',
    title: 'Get the ball in the basket',
    description: 'You only get 2 ramps',
    constraints: {
      availableObjects: ['ramp'],
      maxObjects: 2,
      gravity: 1,
      friction: 0.5,
    },
    setup: {
      objects: [
        { type: 'ball', x: 100, y: 100 },
        { type: 'basket', x: 700, y: 500 },
      ],
    },
    winCondition: 'ball enters basket',
  },
  {
    id: 'tower-survive',
    title: 'Build a tower that survives',
    description: 'Gravity will double after 5 seconds',
    constraints: {
      availableObjects: ['box', 'plank'],
      maxObjects: 6,
      gravity: 1,
      gravityChangesTo: 2,
      gravityChangeDelay: 5000,
    },
    setup: {
      objects: [],
    },
    winCondition: 'tower still standing after gravity change',
  },
];
