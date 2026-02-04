import type Matter from 'matter-js';

export type AppMode = 'scenarios' | 'challenges' | 'sandbox';

export type ScenarioId = 'moon-drop' | 'no-friction' | 'reverse-gravity';
export type ChallengeId = 'ball-in-basket' | 'tower-survive';
export type ObjectType = 'box' | 'ball' | 'ramp' | 'plank' | 'basket' | 'feather';

export interface ScenarioSetup {
  gravity: number;
  friction: number;
  objects: { type: ObjectType; x: number; y: number; angle?: number }[];
}

export interface Scenario {
  id: ScenarioId;
  title: string;
  description: string;
  setup: ScenarioSetup;
  hint: string;
}

export interface ChallengeConstraints {
  availableObjects: ObjectType[];
  maxObjects: number;
  gravity: number;
  friction?: number;
  gravityChangesTo?: number;
  gravityChangeDelay?: number;
}

export interface Challenge {
  id: ChallengeId;
  title: string;
  description: string;
  constraints: ChallengeConstraints;
  setup: { objects: { type: ObjectType; x: number; y: number; angle?: number }[] };
  winCondition: string;
}

export interface DialConfig {
  id: string;
  label: string;
  warning: string;
  min: number;
  max: number;
  default: number;
  unit: string;
}

export interface PaletteItem {
  type: ObjectType;
  label: string;
  icon: string;
}

export interface Explanation {
  title: string;
  text: string;
  diagram: string | null;
}

export type PhysicsBody = Matter.Body;
