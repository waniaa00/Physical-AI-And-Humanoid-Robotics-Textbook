/**
 * Mock for @docusaurus/useDocusaurusContext
 */
module.exports = function useDocusaurusContext() {
  return {
    siteConfig: {
      title: 'Physical AI & Humanoid Robotics',
      tagline: 'A Comprehensive Guide',
      customFields: {
        backendUrl: 'http://localhost:8000',
      },
    },
  };
};
