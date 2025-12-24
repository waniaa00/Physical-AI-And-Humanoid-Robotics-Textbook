/**
 * Mock for @docusaurus/router
 */
module.exports = {
  useHistory: () => ({
    push: jest.fn(),
    replace: jest.fn(),
    go: jest.fn(),
    goBack: jest.fn(),
    goForward: jest.fn(),
  }),
  useLocation: () => ({
    pathname: '/',
    search: '',
    hash: '',
  }),
};
