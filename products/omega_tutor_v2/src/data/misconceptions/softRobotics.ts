/**
 * Soft robotics misconceptions — Port from omega_tutor core/misconception_detector.
 */

import type { Misconception } from "../../services/misconceptions";

export const softRoboticsMisconceptions: Misconception[] = [
  {
    id: "SR001",
    topic: "soft_robotics",
    misconception: "Soft robots are weak and can't apply force",
    correctUnderstanding:
      "Soft robots can apply significant force through pneumatic actuation, tendon drives, or granular jamming. Some can lift many times their weight.",
    commonTriggers: ["soft means weak", "can't grip hard", "no strength"],
    severity: "moderate",
  },
  {
    id: "SR002",
    topic: "soft_robotics",
    misconception: "Soft robots can't be precise",
    correctUnderstanding:
      "With proper sensing and control, soft robots can achieve high precision. Their compliance helps in delicate tasks.",
    commonTriggers: ["not precise", "can't be accurate", "imprecise"],
    severity: "moderate",
  },
];
