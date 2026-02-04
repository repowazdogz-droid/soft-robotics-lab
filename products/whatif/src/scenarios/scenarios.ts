import type { Scenario } from '../types';

export const SCENARIOS: Scenario[] = [
  {
    id: 'moon-drop',
    title: 'What if you were on the Moon?',
    description: 'Drop objects in lunar gravity',
    setup: {
      gravity: 0.16,
      friction: 0.3,
      objects: [
        { type: 'ball', x: 400, y: 100 },
        { type: 'feather', x: 450, y: 100 },
        { type: 'ball', x: 350, y: 100 },
      ],
    },
    hint: 'Watch how differently things fall...',
  },
  {
    id: 'no-friction',
    title: "What if friction didn't exist?",
    description: 'A world where everything slides forever',
    setup: {
      gravity: 1,
      friction: 0,
      objects: [
        { type: 'box', x: 200, y: 300 },
        { type: 'ball', x: 600, y: 300 },
        { type: 'ramp', x: 400, y: 400, angle: -15 },
      ],
    },
    hint: 'Try pushing something...',
  },
  {
    id: 'reverse-gravity',
    title: 'What if gravity flipped?',
    description: 'Everything falls... up',
    setup: {
      gravity: -1,
      friction: 0.5,
      objects: [
        { type: 'box', x: 300, y: 500 },
        { type: 'box', x: 350, y: 500 },
        { type: 'ball', x: 500, y: 500 },
      ],
    },
    hint: 'What happens to a stack of blocks?',
  },
];
