/**
 * Mock for @docusaurus/Link
 */
const React = require('react');

module.exports = function Link({ children, to, ...props }) {
  return React.createElement('a', { href: to, ...props }, children);
};
