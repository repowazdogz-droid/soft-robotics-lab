import type { ScenarioId } from '../types';
import type { Explanation } from '../types';

export const EXPLANATIONS: Record<ScenarioId, Explanation> = {
  'moon-drop': {
    title: 'Lunar gravity',
    text: "The Moon has about 1/6th of Earth's gravity. Objects fall slower, but they still accelerate. A hammer and feather dropped together will land at the same time — there's no air resistance.",
    diagram: null,
  },
  'no-friction': {
    title: 'Frictionless world',
    text: "Friction is what makes things stop. Without it, any push lasts forever. This is Newton's First Law — objects in motion stay in motion unless a force acts on them.",
    diagram: null,
  },
  'reverse-gravity': {
    title: 'Inverted gravity',
    text: "Gravity is just acceleration toward mass. If it reversed, everything would \"fall\" upward at 9.8 m/s². Structures designed for downward gravity would collapse.",
    diagram: null,
  },
};
