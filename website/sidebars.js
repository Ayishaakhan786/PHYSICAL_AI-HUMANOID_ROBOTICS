// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

The sidebars can be generated from filesystem, or explicitly defined here.

Create as many sidebars as you want.

@type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  physicalAiSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsible: false,
      items: [
        'intro/physical-ai-fundamentals',
        'intro/why-humanoid-robotics-matters',
        'intro/how-to-use-this-book',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Physical AI Foundations',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-1-foundations/what-is-physical-ai',
        'modules/module-1-foundations/embodied-intelligence',
        'modules/module-1-foundations/digital-vs-physical-ai',
        'modules/module-1-foundations/simulation-first-workflows',
        'modules/module-1-foundations/module-1-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 Fundamentals',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-2-ros2/ros2-architecture',
        'modules/module-2-ros2/nodes-topics',
        'modules/module-2-ros2/services-actions',
        'modules/module-2-ros2/urdf-robot-descriptions',
        'modules/module-2-ros2/rclpy-basics',
        'modules/module-2-ros2/module-2-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Digital Twin Simulation',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-3-digital-twin/gazebo-concepts',
        'modules/module-3-digital-twin/unity-integration',
        'modules/module-3-digital-twin/sensor-modeling',
        'modules/module-3-digital-twin/environment-modeling',
        'modules/module-3-digital-twin/gazebo-vs-unity-tradeoffs',
        'modules/module-3-digital-twin/module-3-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: NVIDIA Isaac Platform',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-4-isaac/isaac-sim-gpu',
        'modules/module-4-isaac/perception-pipelines',
        'modules/module-4-isaac/vslam-nav2',
        'modules/module-4-isaac/isaac-ros-integration',
        'modules/module-4-isaac/simulation-to-real-training',
        'modules/module-4-isaac/module-4-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: VLA & Conversational Robotics',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-5-vla/vision-language-action',
        'modules/module-5-vla/llm-planning',
        'modules/module-5-vla/speech-models',
        'modules/module-5-vla/conversational-robotics',
        'modules/module-5-vla/module-5-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 6: Humanoid Robotics',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-6-humanoid/humanoid-kinematics',
        'modules/module-6-humanoid/locomotion',
        'modules/module-6-humanoid/human-interaction',
        'modules/module-6-humanoid/humanoid-specific-challenges',
        'modules/module-6-humanoid/module-6-summary',
      ],
    },
    {
      type: 'category',
      label: 'Hardware & Lab Architecture',
      collapsible: true,
      collapsed: false,
      items: [
        'hardware/workstations-jetson',
        'hardware/sensors-robotics',
        'hardware/on-prem-labs',
        'hardware/cloud-labs',
        'hardware/hardware-summary',
      ],
    },
    {
      type: 'category',
      label: 'Sim-to-Real Challenges',
      collapsible: true,
      collapsed: false,
      items: [
        'sim-to-real/domain-randomization',
        'sim-to-real/reality-gap',
        'sim-to-real/transfer-learning',
        'sim-to-real/deployment-constraints',
        'sim-to-real/sim-to-real-summary',
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid',
      collapsible: true,
      collapsed: false,
      items: [
        'capstone/autonomous-humanoid-system',
        'capstone/end-to-end-pipeline',
        'capstone/design-tradeoffs',
        'capstone/real-world-constraints',
        'capstone/future-directions',
        'capstone/capstone-summary',
      ],
    },
  ],
};

export default sidebars;
