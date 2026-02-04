/**
 * Machine Learning 101 — Full curriculum for terrain.
 */

import type { Curriculum } from "../../types/terrain";

export const MACHINE_LEARNING_101: Curriculum = {
  id: "machine-learning-101",
  name: "Machine Learning 101",
  description: "From basics to practice: supervised, unsupervised, and neural nets.",
  estimatedHours: 6,
  topics: [
    {
      id: "ml-basics",
      name: "ML Basics",
      description: "What is machine learning and when to use it.",
      prerequisites: [],
      depth: "intuitive",
      estimatedMinutes: 25,
      keyQuestions: ["What is machine learning?", "How does ML differ from traditional programming?"],
    },
    {
      id: "supervised",
      name: "Supervised Learning",
      description: "Learning from labeled data: regression and classification.",
      prerequisites: ["ml-basics"],
      depth: "structured",
      estimatedMinutes: 45,
      keyQuestions: ["What is supervised learning?", "What is overfitting?"],
    },
    {
      id: "unsupervised",
      name: "Unsupervised Learning",
      description: "Finding structure in unlabeled data.",
      prerequisites: ["ml-basics"],
      depth: "structured",
      estimatedMinutes: 40,
      keyQuestions: ["What is clustering?", "What is dimensionality reduction?"],
    },
    {
      id: "neural-nets",
      name: "Neural Networks",
      description: "From perceptrons to deep learning.",
      prerequisites: ["supervised"],
      depth: "technical",
      estimatedMinutes: 55,
      keyQuestions: ["What is a neural network?", "How does backpropagation work?"],
    },
  ],
};
