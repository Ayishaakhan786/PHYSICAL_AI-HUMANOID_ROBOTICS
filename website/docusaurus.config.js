// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Spec-Driven Guide to Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physicalai-humanoid-robotics.github.io',
  // Set /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'PhysicalAI-Humanoid-Robotics',
  projectName: 'PHYSICALAI_HUMANOID_ROBOTICS',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl:
            'https://github.com/PhysicalAI-Humanoid-Robotics/PHYSICALAI_HUMANOID_ROBOTICS/edit/main/website/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'physicalAiSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/PhysicalAI-Humanoid-Robotics/PHYSICALAI_HUMANOID_ROBOTICS',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Getting Started',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro/physical-ai-fundamentals',
              },
              {
                label: 'How to Use This Book',
                to: '/docs/intro/how-to-use-this-book',
              },
            ],
          },
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: Physical AI Foundations',
                to: '/docs/modules/module-1-foundations/what-is-physical-ai',
              },
              {
                label: 'Module 2: ROS 2 Fundamentals',
                to: '/docs/modules/module-2-ros2/ros2-architecture',
              },
              {
                label: 'Module 3: Digital Twin Simulation',
                to: '/docs/modules/module-3-digital-twin/gazebo-concepts',
              },
              {
                label: 'Module 4: NVIDIA Isaac Platform',
                to: '/docs/modules/module-4-isaac/isaac-sim-gpu',
              },
              {
                label: 'Module 5: VLA & Conversational Robotics',
                to: '/docs/modules/module-5-vla/vision-language-action',
              },
              {
                label: 'Module 6: Humanoid Robotics',
                to: '/docs/modules/module-6-humanoid/humanoid-kinematics',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/PhysicalAI-Humanoid-Robotics/PHYSICALAI_HUMANOID_ROBOTICS',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml', 'cpp'],
      },
    }),
};

export default config;
