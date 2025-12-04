import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    'book-structure',
    {
      type: 'category',
      label: '📘 Module 1: Foundations (Weeks 1-4)',
      collapsed: false,
      items: [
        'module1/chapter1-introduction',
        'module1/chapter2-ros2-fundamentals',
        'module1/chapter3-kinematics',
        'module1/chapter4-dynamics-control',
      ],
    },
    {
      type: 'category',
      label: '🎮 Module 2: Simulation & Perception (Weeks 5-8)',
      collapsed: false,
      items: [
        'module2/chapter5-gazebo-simulation',
        'module2/chapter6-isaac-sim',
        'module2/chapter7-computer-vision',
        'module2/chapter8-slam-navigation',
      ],
    },
    {
      type: 'category',
      label: '🤖 Module 3: Humanoid Robotics (Weeks 9-12)',
      collapsed: false,
      items: [
        'module3/chapter9-bipedal-locomotion',
        'module3/chapter10-manipulation',
        'module3/chapter11-whole-body-control',
        'module3/chapter12-hri',
      ],
    },
    {
      type: 'category',
      label: '🧠 Module 4: AI Integration (Weeks 13-16)',
      collapsed: false,
      items: [
        'module4/chapter13-vla-models',
        'module4/chapter14-llm-planning',
        'module4/chapter15-integration',
        'module4/chapter16-deployment',
      ],
    },
  ],
};

export default sidebars;
