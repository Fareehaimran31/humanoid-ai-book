import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

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
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 – Robotic Nervous System',
      items: [
        'module-1-ros/chapter-1-introduction',
        'module-1-ros/chapter-2-communication',
        'module-1-ros/chapter-3-practical-labs',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins – Gazebo & Unity',
      items: [
        'module-2-digital-twins/chapter-1-gazebo-basics',
        'module-2-digital-twins/chapter-2-unity-simulation',
        'module-2-digital-twins/chapter-3-comparison',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI Robot Brain – NVIDIA Isaac Lab',
      items: [
        'module-3-ai-brain/chapter-1-isaac-lab-overview',
        'module-3-ai-brain/chapter-2-ai-integration',
        'module-3-ai-brain/chapter-3-intelligent-behaviors',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Systems',
      items: [
        'module-4-vla/chapter-1-vision-systems',
        'module-4-vla/chapter-2-language-processing',
        'module-4-vla/chapter-3-vla-integration',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project: Autonomous Humanoid Robot',
      items: [
        'capstone-project/autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;
