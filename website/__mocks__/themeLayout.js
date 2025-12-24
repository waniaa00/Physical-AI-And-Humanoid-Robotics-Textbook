/**
 * Mock for @theme/Layout
 */
const React = require('react');

module.exports = function Layout({ children, title, description }) {
  return React.createElement('div', { 'data-testid': 'layout', title, description }, children);
};
