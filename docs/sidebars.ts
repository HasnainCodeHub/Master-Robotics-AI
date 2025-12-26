import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Physical AI & Humanoid Robotics Textbook Sidebar Configuration
 *
 * Structure mirrors the locked curriculum specifications from Phase 1.
 * Module order: M1 → M2 → M3 → M4 → M5 → Capstone
 */
const sidebars: SidebarsConfig = {
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Physical AI Foundations',
      link: {
        type: 'doc',
        id: 'module-1/index',
      },
      items: [
        'module-1/chapter-1-embodied-intelligence',
        'module-1/chapter-2-sensor-fundamentals',
        'module-1/chapter-3-actuator-fundamentals',
        'module-1/chapter-4-physical-constraints',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 Fundamentals',
      link: {
        type: 'doc',
        id: 'module-2/index',
      },
      items: [
        'module-2/chapter-1-ros2-architecture',
        'module-2/chapter-2-topics-pubsub',
        'module-2/chapter-3-services',
        'module-2/chapter-4-actions',
        'module-2/chapter-5-urdf-robot-description',
        'module-2/chapter-6-integration-patterns',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Simulation & Digital Twin',
      link: {
        type: 'doc',
        id: 'module-3/module-3-index',
      },
      items: [
        'module-3/chapter-1-simulation-fundamentals',
        'module-3/chapter-2-gazebo-world-building',
        'module-3/chapter-3-simulated-sensors',
        'module-3/chapter-4-simulated-actuators',
        'module-3/chapter-5-unity-integration',
        'module-3/chapter-6-reality-gap',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: NVIDIA Isaac Ecosystem',
      link: {
        type: 'doc',
        id: 'module-4/module-4-index',
      },
      items: [
        'module-4/chapter-1-isaac-sim-foundations',
        'module-4/chapter-2-scene-composition-sensors',
        'module-4/chapter-3-isaac-ros-integration',
        'module-4/chapter-4-visual-slam',
        'module-4/chapter-5-domain-randomization',
        'module-4/chapter-6-sim-to-real-transfer',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'module-5/module-5-index',
      },
      items: [
        'module-5/chapter-1-speech-recognition',
        'module-5/chapter-2-llm-task-planning',
        'module-5/chapter-3-grounding',
        'module-5/chapter-4-vision-language-models',
        'module-5/chapter-5-safety-constraints',
        'module-5/chapter-6-failure-handling',
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Integrated Humanoid System',
      link: {
        type: 'doc',
        id: 'capstone/capstone-index',
      },
      items: [
        'capstone/chapter-1-system-architecture',
        'capstone/chapter-2-robot-setup',
        'capstone/chapter-3-navigation',
        'capstone/chapter-4-manipulation',
        'capstone/chapter-5-integration',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      link: {
        type: 'doc',
        id: 'appendix/appendix-index',
      },
      items: [
        'appendix/ml-concepts-roboticists',
        'appendix/hardware-requirements',
        'appendix/ros2-installation',
        'appendix/simulation-installation',
      ],
    },
    'glossary',
  ],
};

export default sidebars;
