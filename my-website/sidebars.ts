import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [
        'intro/index',
        'intro/book-structure',
        'intro/physical-ai',
        'intro/embodied-intelligence',
        'intro/sim-to-real',
        'intro/skills-mindset',
        'intro/code-examples',
        'intro/cross-references',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Foundation',
      collapsed: false,
      items: [
        'module-1-foundation/chapter1-mathematical-foundations',
        'module-1-foundation/chapter2-kinematics-dynamics',
        'module-1-foundation/chapter3-sensing-perception',
        'module-1-foundation/chapter4-embodied-intelligence',
        'module-1-foundation/summary',
        'module-1-foundation/exercises',
        'module-1-foundation/cross-references',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Robotic Nervous System',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Architecture',
          items: [
            'module-2-robotic-nervous-system/chapter-1/ros2-architecture',
            'module-2-robotic-nervous-system/chapter-1/exercises',
            'module-2-robotic-nervous-system/chapter-1/solutions',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: AI Agent Bridge',
          items: [
            'module-2-robotic-nervous-system/chapter-2/ai-agent-bridge',
            'module-2-robotic-nervous-system/chapter-2/exercises',
            'module-2-robotic-nervous-system/chapter-2/solutions',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: URDF Humanoid Description',
          items: [
            'module-2-robotic-nervous-system/chapter-3/urdf-humanoid-description',
            'module-2-robotic-nervous-system/chapter-3/exercises',
            'module-2-robotic-nervous-system/chapter-3/solutions',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Robotic Nervous System Patterns',
          items: [
            'module-2-robotic-nervous-system/chapter-4/robotic-nervous-system-patterns',
            'module-2-robotic-nervous-system/chapter-4/exercises',
            'module-2-robotic-nervous-system/chapter-4/solutions',
          ],

        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Gazebo Simulation Environment Setup',
          items: [
            'module-3-digital-twin/chapter-1-gazebo-basics/content',
            'module-3-digital-twin/chapter-1-gazebo-basics/exercises',
            'module-3-digital-twin/chapter-1-gazebo-basics/solutions',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: URDF and SDF Robot Description Formats',
          items: [
            'module-3-digital-twin/chapter-2-robot-modeling/content',
            'module-3-digital-twin/chapter-2-robot-modeling/exercises',
            'module-3-digital-twin/chapter-2-robot-modeling/solutions',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Physics Simulation and Sensor Simulation',
          items: [
            'module-3-digital-twin/chapter-3-sensor-simulation/content',
            'module-3-digital-twin/chapter-3-sensor-simulation/exercises',
            'module-3-digital-twin/chapter-3-sensor-simulation/solutions',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Unity Visualization and Interaction',
          items: [
            'module-3-digital-twin/chapter-4-unity-integration/content',
            'module-3-digital-twin/chapter-4-unity-integration/exercises',
            'module-3-digital-twin/chapter-4-unity-integration/solutions',
          ],
        },
      ],
    },
    // Additional modules will be added as they are created
  ],
};

export default sidebars;
