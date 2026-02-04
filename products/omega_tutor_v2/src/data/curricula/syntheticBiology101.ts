/**
 * Synthetic Biology 101 — Full curriculum for terrain.
 */

import type { Curriculum } from "../../types/terrain";

export const SYNTHETIC_BIOLOGY_101: Curriculum = {
  id: "synthetic-biology-101",
  name: "Synthetic Biology 101",
  description: "From DNA to systems: genetic circuits and gene expression.",
  estimatedHours: 5,
  topics: [
    {
      id: "sb-basics",
      name: "Synthetic Biology Basics",
      description: "What is synthetic biology and its design principles.",
      prerequisites: [],
      depth: "intuitive",
      estimatedMinutes: 25,
      keyQuestions: ["What is synthetic biology?", "What is a genetic circuit?"],
    },
    {
      id: "genetic-circuits",
      name: "Genetic Circuits",
      description: "Designing logic and regulation in living cells.",
      prerequisites: ["sb-basics"],
      depth: "structured",
      estimatedMinutes: 45,
      keyQuestions: ["How do genetic circuits work?", "What is a toggle switch?"],
    },
    {
      id: "gene-expression",
      name: "Gene Expression",
      description: "Transcription, translation, and regulation.",
      prerequisites: ["sb-basics"],
      depth: "structured",
      estimatedMinutes: 40,
      keyQuestions: ["How is gene expression controlled?", "What is a promoter?"],
    },
  ],
};
