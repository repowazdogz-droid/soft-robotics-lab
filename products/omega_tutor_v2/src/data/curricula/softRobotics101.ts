/**
 * Soft Robotics 101 — Full curriculum for terrain.
 */

import type { Curriculum } from "../../types/terrain";

export const SOFT_ROBOTICS_101: Curriculum = {
  id: "soft-robotics-101",
  name: "Soft Robotics 101",
  description:
    "Foundations of soft robotics: materials, actuation, sensing, and control.",
  estimatedHours: 8,
  topics: [
    {
      id: "sr-intro",
      name: "Introduction to Soft Robotics",
      description: "What are soft robots and why do they matter?",
      prerequisites: [],
      depth: "intuitive",
      estimatedMinutes: 20,
      keyQuestions: [
        "What is a soft robot?",
        "How do soft robots differ from rigid robots?",
      ],
    },
    {
      id: "sr-materials",
      name: "Soft Materials",
      description: "Silicones, hydrogels, and other materials for soft robots.",
      prerequisites: ["sr-intro"],
      depth: "structured",
      estimatedMinutes: 30,
      keyQuestions: [
        "What materials are used in soft robotics?",
        "What is DragonSkin silicone?",
      ],
    },
    {
      id: "sr-actuation",
      name: "Soft Actuation",
      description:
        "Pneumatic, hydraulic, cable-driven, and smart material actuators.",
      prerequisites: ["sr-materials"],
      depth: "structured",
      estimatedMinutes: 45,
      keyQuestions: [
        "How do pneumatic actuators work?",
        "What is a McKibben muscle?",
      ],
    },
    {
      id: "sr-morphological",
      name: "Morphological Computation",
      description: "Using body structure for computation.",
      prerequisites: ["sr-actuation"],
      depth: "technical",
      estimatedMinutes: 40,
      keyQuestions: [
        "What is morphological computation?",
        "How can body shape perform computation?",
      ],
    },
    {
      id: "sr-sensing",
      name: "Soft Sensing",
      description: "Embedded sensors, proprioception, and feedback.",
      prerequisites: ["sr-materials"],
      depth: "structured",
      estimatedMinutes: 35,
      keyQuestions: [
        "How do soft robots sense their environment?",
        "What is proprioception?",
      ],
    },
    {
      id: "sr-control",
      name: "Control of Soft Robots",
      description: "Challenges and approaches to controlling soft systems.",
      prerequisites: ["sr-actuation", "sr-sensing"],
      depth: "technical",
      estimatedMinutes: 50,
      keyQuestions: [
        "Why is controlling soft robots difficult?",
        "What control strategies work?",
      ],
    },
    {
      id: "sr-applications",
      name: "Applications",
      description: "Medical, manipulation, locomotion, and wearables.",
      prerequisites: ["sr-control"],
      depth: "structured",
      estimatedMinutes: 30,
      keyQuestions: [
        "Where are soft robots used?",
        "What are soft robotic grippers?",
      ],
    },
    {
      id: "sr-future",
      name: "Future Directions",
      description: "Open problems and research frontiers.",
      prerequisites: ["sr-applications"],
      depth: "research",
      estimatedMinutes: 25,
      keyQuestions: [
        "What are the open problems in soft robotics?",
        "Where is the field heading?",
      ],
    },
  ],
};
