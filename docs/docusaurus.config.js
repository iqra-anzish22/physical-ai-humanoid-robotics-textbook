const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').DocusaurusConfig} */
(module.exports = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive textbook on Physical AI & Humanoid Robotics for advanced students and educators',

  // ⭐ GitHub Pages URL Settings
  url: 'https://iqra-anzish22.github.io',
  baseUrl: '/physical-ai-humanoid-robotics-textbook/',
  organizationName: 'iqra-anzish22',
  projectName: 'physical-ai-humanoid-robotics-textbook',
  

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',

  presets: [
    [
      '@docusaurus/preset-classic',
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),

          // ⭐ Edit button GitHub link
          editUrl:
            'https://github.com/iqra-anzish22/physical-ai-humanoid-robotics-textbook/edit/main/',

          // ⭐ Serve docs at site root
          routeBasePath: '/',

          sidebarCollapsible: true,
        },
        blog: false, // Blog disabled
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
        gtag: {
          trackingID: 'G-XXXXXXXXXX',
          anonymizeIP: true,
        },
      }),
    ],
  ],

  // ⭐ Language Settings
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },

  scripts: [
    {
      src: '/static/chatbot.js',
      async: true,
      defer: true,
    },
  ],
  themeConfig: ({
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Textbook Logo',
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
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/iqra-anzish22/physical-ai-humanoid-robotics-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            { label: 'Introduction', to: '/intro' },
            { label: 'Physical AI Concepts', to: '/physical-ai/index' },
            { label: 'ROS 2', to: '/ros2/index' },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/iqra-anzish22/physical-ai-humanoid-robotics-textbook',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Education. Built with Docusaurus.`,
    },

    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
  }),
});
