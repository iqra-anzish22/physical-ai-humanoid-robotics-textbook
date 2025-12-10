/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

module.exports = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Physical AI Concepts',
      items: [
        'physical-ai/index',
        'physical-ai/concepts',
        'physical-ai/exercises'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'ROS 2',
      items: [
        'ros2/index',
        'ros2/setup',
        'ros2/examples'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Gazebo & Unity',
      items: [
        'gazebo-unity/index',
        'gazebo-unity/simulation',
        'gazebo-unity/examples'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac',
      items: [
        'nvidia-isaac/index',
        'nvidia-isaac/setup',
        'nvidia-isaac/examples'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Humanoid Robotics',
      items: [
        'humanoid-robotics/index',
        'humanoid-robotics/concepts',
        'humanoid-robotics/exercises'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'VLA Basics',
      items: [
        'vla-basics/index',
        'vla-basics/applications',
        'vla-basics/exercises'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        'tutorial-basics/creating-content',
        'tutorial-basics/exercises'
      ],
      collapsed: true,
    }
  ],
};
