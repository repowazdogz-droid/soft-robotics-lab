import type { DialConfig, PaletteItem } from './types';

export const DIALS: DialConfig[] = [
  {
    id: 'gravity',
    label: 'Gravity',
    warning: '⚠ unstable at extremes',
    min: -2,
    max: 5,
    default: 1,
    unit: 'g',
  },
  {
    id: 'friction',
    label: 'Friction',
    warning: '⚠ things get slippery',
    min: 0,
    max: 1,
    default: 0.5,
    unit: '',
  },
  {
    id: 'timeScale',
    label: 'Time',
    warning: '⚠ experimental',
    min: 0.1,
    max: 3,
    default: 1,
    unit: 'x',
  },
];

export const OBJECTS: PaletteItem[] = [
  { type: 'box', label: 'Box', icon: '📦' },
  { type: 'ball', label: 'Ball', icon: '⚽' },
  { type: 'ramp', label: 'Ramp', icon: '📐' },
  { type: 'plank', label: 'Plank', icon: '🪵' },
];
