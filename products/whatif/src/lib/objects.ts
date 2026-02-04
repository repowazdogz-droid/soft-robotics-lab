import Matter from 'matter-js';
import type { ObjectType } from '../types';

const COLORS = {
  box: '#ff6b6b',
  ball: '#4ecdc4',
  ramp: '#ffe66d',
  plank: '#95e1d3',
  basket: '#a8e6cf',
  feather: '#dda0dd',
} as const;

export function createBox(x: number, y: number, friction = 0.5): Matter.Body {
  return Matter.Bodies.rectangle(x, y, 50, 50, {
    render: { fillStyle: COLORS.box },
    friction,
    restitution: 0.3,
  });
}

export function createBall(x: number, y: number, friction = 0.5): Matter.Body {
  return Matter.Bodies.circle(x, y, 25, {
    render: { fillStyle: COLORS.ball },
    friction,
    restitution: 0.8,
  });
}

export function createRamp(
  x: number,
  y: number,
  angle = -20,
  friction = 0.5
): Matter.Body {
  return Matter.Bodies.rectangle(x, y, 200, 20, {
    isStatic: true,
    angle: (angle * Math.PI) / 180,
    render: { fillStyle: COLORS.ramp },
    friction,
  });
}

export function createPlank(x: number, y: number, friction = 0.5): Matter.Body {
  return Matter.Bodies.rectangle(x, y, 150, 20, {
    render: { fillStyle: COLORS.plank },
    friction,
    restitution: 0.1,
  });
}

export function createBasket(x: number, y: number): Matter.Body {
  return Matter.Bodies.rectangle(x, y, 80, 60, {
    isStatic: true,
    render: { fillStyle: COLORS.basket },
    friction: 0.5,
  });
}

export function createFeather(x: number, y: number, friction = 0.5): Matter.Body {
  return Matter.Bodies.circle(x, y, 15, {
    render: { fillStyle: COLORS.feather },
    friction,
    restitution: 0.5,
    density: 0.001,
  });
}

export function createObject(
  type: ObjectType,
  x: number,
  y: number,
  opts: { angle?: number; friction?: number } = {}
): Matter.Body {
  const { angle = -20, friction = 0.5 } = opts;
  switch (type) {
    case 'box':
      return createBox(x, y, friction);
    case 'ball':
      return createBall(x, y, friction);
    case 'ramp':
      return createRamp(x, y, angle, friction);
    case 'plank':
      return createPlank(x, y, friction);
    case 'basket':
      return createBasket(x, y);
    case 'feather':
      return createFeather(x, y, friction);
    default:
      return createBox(x, y, friction);
  }
}
