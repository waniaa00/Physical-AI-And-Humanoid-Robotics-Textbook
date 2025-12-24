/**
 * Component tests for InterestPrompt (T031)
 * Tests empty state when user has no selected interests
 *
 * Test cases:
 * - Renders friendly prompt message
 * - Shows "Select Interests" button linking to profile
 * - Shows "Browse All Content" button linking to docs
 * - Displays helpful description text
 * - Contains proper styling/classes for visual appeal
 */

import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';

// Mock Docusaurus Link - preserve className and other props
jest.mock('@docusaurus/Link', () => {
  return function MockLink({ children, to, className, ...props }: any) {
    return <a href={to} className={className} {...props}>{children}</a>;
  };
});

import InterestPrompt from '@site/src/components/PersonalizedContent/InterestPrompt';

describe('InterestPrompt Component', () => {
  it('should render the component', () => {
    render(<InterestPrompt />);

    // Component should be rendered
    expect(screen.getByTestId('interest-prompt')).toBeInTheDocument();
  });

  it('should display friendly heading message', () => {
    render(<InterestPrompt />);

    // Should have a welcoming heading
    expect(
      screen.getByRole('heading', { name: /no interests selected|select your interests/i })
    ).toBeInTheDocument();
  });

  it('should display helpful description text', () => {
    render(<InterestPrompt />);

    // Should explain what the user should do
    expect(
      screen.getByText(/tell us what you're interested in|personalize your learning/i)
    ).toBeInTheDocument();
  });

  it('should render "Select Interests" link to profile page', () => {
    render(<InterestPrompt />);

    // Find the link by its text content
    const selectInterestsLink = screen.getByRole('link', { name: /select interests/i });

    expect(selectInterestsLink).toBeInTheDocument();
    expect(selectInterestsLink).toHaveAttribute('href', '/profile');
  });

  it('should render "Browse All Content" link to docs', () => {
    render(<InterestPrompt />);

    // Find the browse all content link
    const browseAllLink = screen.getByRole('link', { name: /browse all content/i });

    expect(browseAllLink).toBeInTheDocument();
    expect(browseAllLink).toHaveAttribute('href', '/docs/intro');
  });

  it('should have primary styling for Select Interests button', () => {
    render(<InterestPrompt />);

    const selectInterestsLink = screen.getByRole('link', { name: /select interests/i });

    // Should have button class (identity-obj-proxy returns the key as the value)
    expect(selectInterestsLink.className).toBeTruthy();
    expect(selectInterestsLink.className.length).toBeGreaterThan(0);
  });

  it('should have secondary styling for Browse All Content button', () => {
    render(<InterestPrompt />);

    const browseAllLink = screen.getByRole('link', { name: /browse all content/i });

    // Should have button class (identity-obj-proxy returns the key as the value)
    expect(browseAllLink.className).toBeTruthy();
    expect(browseAllLink.className.length).toBeGreaterThan(0);
  });

  it('should display icon or emoji for visual appeal', () => {
    render(<InterestPrompt />);

    // Should contain some visual elements (emojis or icons)
    const container = screen.getByTestId('interest-prompt');

    // Check for common icon patterns or emojis
    expect(container.textContent).toMatch(/[📚🎯✨🔍]|icon/i);
  });

  it('should have centered layout', () => {
    render(<InterestPrompt />);

    const container = screen.getByTestId('interest-prompt');

    // Should have container class (CSS modules are mocked)
    expect(container.className).toBeTruthy();
    expect(container).toBeInTheDocument();
  });

  it('should display both action buttons in correct order', () => {
    render(<InterestPrompt />);

    const links = screen.getAllByRole('link');
    const linkTexts = links.map(link => link.textContent);

    // Select Interests should come before Browse All Content
    const selectIndex = linkTexts.findIndex(text => /select interests/i.test(text || ''));
    const browseIndex = linkTexts.findIndex(text => /browse all/i.test(text || ''));

    expect(selectIndex).toBeGreaterThanOrEqual(0);
    expect(browseIndex).toBeGreaterThanOrEqual(0);
    expect(selectIndex).toBeLessThan(browseIndex);
  });

  it('should have accessible button labels', () => {
    render(<InterestPrompt />);

    // Buttons should have clear, accessible text
    const selectButton = screen.getByRole('link', { name: /select interests/i });
    const browseButton = screen.getByRole('link', { name: /browse all/i });

    // Text should be visible (not empty or just whitespace)
    expect(selectButton.textContent?.trim().length).toBeGreaterThan(5);
    expect(browseButton.textContent?.trim().length).toBeGreaterThan(5);
  });

  it('should provide clear call-to-action', () => {
    render(<InterestPrompt />);

    const container = screen.getByTestId('interest-prompt');

    // Should contain motivating text
    expect(container.textContent).toMatch(/get started|personalize|customize|choose/i);
  });
});
