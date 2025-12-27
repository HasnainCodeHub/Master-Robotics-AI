import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Textbook for Building Intelligent Embodied Systems',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://hasnaincodehub.github.io',
  baseUrl: '/Master-Robotics-AI/',

  organizationName: 'HasnainCodeHub',
  projectName: 'Master-Robotics-AI',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Custom fields accessible via useDocusaurusContext()
  // These are safe to use in frontend code (process.env is only used here in Node.js)
  customFields: {
    apiBaseUrl: process.env.API_BASE_URL || 'https://master-robotics-backend-production.up.railway.app',
    isProduction: process.env.NODE_ENV === 'production',
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: 'textbook',
          editUrl: 'https://github.com/physical-ai-robotics/robotics-book/tree/main/docs/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/physical-ai-social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'custom-search',
          position: 'left',
        },
        {
          type: 'custom-auth',
          position: 'right',
        },
        {
          href: 'https://github.com/HasnainCodeHub/Master-Robotics-AI',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Curriculum',
          items: [
            {
              label: 'Module 1: Physical AI Foundations',
              to: '/textbook/module-1',
            },
            {
              label: 'Module 2: ROS 2 Fundamentals',
              to: '/textbook/module-2',
            },
            {
              label: 'Module 3: Simulation & Digital Twin',
              to: '/textbook/module-3',
            },
          ],
        },
        {
          title: 'Advanced',
          items: [
            {
              label: 'Module 4: NVIDIA Isaac Ecosystem',
              to: '/textbook/module-4',
            },
            {
              label: 'Module 5: Vision-Language-Action',
              to: '/textbook/module-5',
            },
            {
              label: 'Capstone Project',
              to: '/textbook/capstone',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Glossary',
              to: '/textbook/glossary',
            },
            {
              label: 'Appendices',
              to: '/textbook/appendix',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/HasnainCodeHub/Master-Robotics-AI',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json'],
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
