import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Building Intelligent Physical Systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://localhost',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config (will be configured later)
  organizationName: 'HP', // Will be updated for GitHub Pages
  projectName: 'humanoid-robotics', // Project repository name

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  customFields: {
    backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Edit URL removed - will be added when GitHub repo is configured
          editUrl: undefined,
        },
        blog: false, // Disable blog for book-focused site
        theme: {
          customCss: ['./src/css/custom.css', './src/css/tailwind.css'],
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-client-redirects',
      {
        fromExtensions: ['html'],
      },
    ],
  ],

  themes: [],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Robot Logo',
        src: 'img/robot-logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Chapters',
        },
        {
          // T044: Link to personalized content page
          to: '/personalized-content',
          label: '✨ My Content',
          position: 'left',
        },
        {
          type: 'docsVersionDropdown',
          position: 'right',
        },
        {
          to: '/profile',
          label: '👤 Profile',
          position: 'right',
        },
        {
          href: 'https://github.com/HP/humanoid-robotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Chapters',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'Isaac Sim',
              href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/',
            },
            {
              label: 'Unity Robotics Hub',
              href: 'https://github.com/Unity-Technologies/Unity-Robotics-Hub',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/HP/humanoid-robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'csharp', 'cpp', 'markup'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
