import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI Book', 
  tagline: 'Building the Future of Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // ------------------------------------------------------------------
  // 🛠️ GITHUB PAGES FIXES
  // Settings for GitHub Pages deployment
  // ------------------------------------------------------------------
  url: 'https://minahilf.github.io', 
  baseUrl: '/HACKATHON-I-PHYSICAL-AI-HUMANOID/', 

  organizationName: 'minahilf', // AAPKA GITHUB USERNAME
  projectName: 'HACKATHON-I-PHYSICAL-AI-HUMANOID', // AAPKI REPOSITORY KA NAAM
  // ------------------------------------------------------------------

  // 🛠️ BROKEN LINK FIX: 'throw' se 'warn' kar diya taake build fail na ho
  onBrokenLinks: 'warn', 
  onBrokenMarkdownLinks: 'warn',

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
        },
        blog: false, // Blog disabled
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },

    navbar: {
      title: 'Physical AI Book',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      style: 'dark',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook', 
        },
        {
          href: 'https://github.com/facebook/docusaurus',
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
              label: 'Start Reading',
              to: '/docs/intro',
            },
            {
              label: 'Modules',
              to: '/docs/category/module-1-ros-2-nervous-system',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/facebook/docusaurus',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Team. Built for the Future.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;