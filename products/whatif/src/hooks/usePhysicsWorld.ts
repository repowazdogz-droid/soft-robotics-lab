import { useEffect, useRef, useCallback } from 'react';
import Matter from 'matter-js';
import { useStore } from '../store';
import { createObject } from '../lib/objects';
import type { ObjectType } from '../types';

const GROUND_HEIGHT = 60;
const BACKGROUND = '#1a1a2e';

export function usePhysicsWorld(
  canvasRef: React.RefObject<HTMLCanvasElement | null>,
  containerRef: React.RefObject<HTMLDivElement | null>
) {
  const engineRef = useRef<Matter.Engine | null>(null);
  const renderRef = useRef<Matter.Render | null>(null);
  const groundRef = useRef<Matter.Body | null>(null);
  const rafRef = useRef<number>(0);

  const gravity = useStore((s) => s.gravity);
  const friction = useStore((s) => s.friction);
  const timeScale = useStore((s) => s.timeScale);
  const isPaused = useStore((s) => s.isPaused);
  const timeScaleRef = useRef(timeScale);
  const isPausedRef = useRef(isPaused);
  timeScaleRef.current = timeScale;
  isPausedRef.current = isPaused;

  const getWidth = useCallback(() => {
    if (!containerRef?.current) return 800;
    return containerRef.current.clientWidth;
  }, [containerRef]);

  const getHeight = useCallback(() => {
    if (!containerRef?.current) return 600;
    return containerRef.current.clientHeight;
  }, [containerRef]);

  useEffect(() => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    const width = container.clientWidth;
    const height = container.clientHeight;
    canvas.width = width;
    canvas.height = height;

    const engine = Matter.Engine.create();
    engine.gravity.y = gravity;

    const render = Matter.Render.create({
      canvas,
      engine,
      options: {
        width,
        height,
        wireframes: false,
        background: BACKGROUND,
        pixelRatio: Math.min(window.devicePixelRatio, 2),
      },
    });
    Matter.Render.run(render);

    const ground = Matter.Bodies.rectangle(
      width / 2,
      height - GROUND_HEIGHT / 2,
      width + 100,
      GROUND_HEIGHT,
      {
        isStatic: true,
        render: { fillStyle: '#333' },
      }
    );
    Matter.Composite.add(engine.world, ground);

    engineRef.current = engine;
    renderRef.current = render;
    groundRef.current = ground;

    let last = performance.now();
    function loop(now: number) {
      rafRef.current = requestAnimationFrame(loop);
      const delta = Math.min(now - last, 50);
      last = now;
      if (!isPausedRef.current && engineRef.current) {
        Matter.Engine.update(engineRef.current, delta * timeScaleRef.current);
      }
    }
    rafRef.current = requestAnimationFrame(loop);

    return () => {
      cancelAnimationFrame(rafRef.current);
      Matter.Render.stop(render);
      Matter.Composite.clear(engine.world, false);
      Matter.Engine.clear(engine);
      renderRef.current = null;
      engineRef.current = null;
      groundRef.current = null;
    };
  }, []);

  useEffect(() => {
    if (engineRef.current) engineRef.current.gravity.y = gravity;
  }, [gravity]);

  useEffect(() => {
    if (!engineRef.current) return;
    const world = engineRef.current.world;
    Matter.Composite.allBodies(world).forEach((body) => {
      if (body.isStatic) return;
      body.friction = friction;
      body.frictionAir = 0.01 * (1 - friction * 0.5);
    });
  }, [friction]);

  const addBody = useCallback((body: Matter.Body) => {
    if (engineRef.current) {
      Matter.Composite.add(engineRef.current.world, body);
    }
  }, []);

  const addObject = useCallback(
    (type: ObjectType, x: number, y: number, opts?: { angle?: number }) => {
      if (!engineRef.current) return null;
      const body = createObject(type, x, y, {
        friction: useStore.getState().friction,
        angle: opts?.angle,
      });
      Matter.Composite.add(engineRef.current.world, body);
      return body;
    },
    []
  );

  const clear = useCallback(() => {
    const engine = engineRef.current;
    if (!engine) return;
    const world = engine.world;
    const toRemove = Matter.Composite.allBodies(world).filter(
      (b) => b !== groundRef.current
    );
    Matter.Composite.remove(world, toRemove);
  }, []);

  return {
    engine: engineRef.current,
    addBody,
    addObject,
    clear,
    getWidth,
    getHeight,
  };
}
