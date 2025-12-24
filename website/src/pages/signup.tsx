import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { InterestSelector } from '@site/src/components/Interests';
import { setCurrentUserId, setUserHasInterests } from '../lib/user';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './auth.module.css';

type SignUpStep = 'account' | 'interests';

export default function SignUp() {
  const { signUp } = useAuth(); // T029: Integrate AuthContext
  const [currentStep, setCurrentStep] = useState<SignUpStep>('account');
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    confirmPassword: '',
    background: 'student' as 'student' | 'professional',
    languagePreference: 'en' as 'en' | 'ur',
  });
  const [selectedInterests, setSelectedInterests] = useState<number[]>([]);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value,
    });
  };

  const handleAccountSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    // T031: Client-side validation
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match!');
      return;
    }

    // T031: Password strength validation
    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters long');
      return;
    }

    // T031: Email format validation (basic)
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(formData.email)) {
      setError('Please enter a valid email address');
      return;
    }

    // T031: Password strength - check for uppercase, lowercase, number
    const hasUppercase = /[A-Z]/.test(formData.password);
    const hasLowercase = /[a-z]/.test(formData.password);
    const hasNumber = /[0-9]/.test(formData.password);

    if (!hasUppercase || !hasLowercase || !hasNumber) {
      setError('Password must contain at least one uppercase letter, one lowercase letter, and one number');
      return;
    }

    // Move to interests step
    setCurrentStep('interests');
  };

  const handleCompleteSignup = async () => {
    setIsSubmitting(true);
    setError(null);

    // T031: Validate interests selection (2-5 interests)
    if (selectedInterests.length < 2 || selectedInterests.length > 5) {
      setError('Please select between 2 and 5 interests');
      setIsSubmitting(false);
      return;
    }

    try {
      // T029: Use AuthContext signUp method
      await signUp({
        email: formData.email,
        password: formData.password,
        interests: selectedInterests,
        background: formData.background,
        language_preference: formData.languagePreference,
      });

      // Mark user as having interests for personalization
      setUserHasInterests(true);

      // T032: Redirect to chat page after successful sign-up
      console.log('Sign up completed successfully');
      window.location.href = '/docs/intro';
    } catch (err) {
      // T029: Display validation errors
      const errorMessage = err instanceof Error ? err.message : 'Registration failed';
      setError(errorMessage);
      console.error('Sign up error:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleBackToAccount = () => {
    setCurrentStep('account');
  };

  const isInterestsValid = selectedInterests.length >= 2 && selectedInterests.length <= 5;

  return (
    <Layout title="Sign Up" description="Create an account to access the Physical AI & Humanoid Robotics course">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          {/* Progress Indicator */}
          <div className={styles.progressIndicator}>
            <div className={`${styles.progressStep} ${currentStep === 'account' ? styles.active : styles.completed}`}>
              <div className={styles.stepNumber}>1</div>
              <div className={styles.stepLabel}>Account</div>
            </div>
            <div className={styles.progressLine}></div>
            <div className={`${styles.progressStep} ${currentStep === 'interests' ? styles.active : ''}`}>
              <div className={styles.stepNumber}>2</div>
              <div className={styles.stepLabel}>Interests</div>
            </div>
          </div>

          {/* Step 1: Account Details */}
          {currentStep === 'account' && (
            <>
              <div className={styles.authHeader}>
                <div className={styles.robotIcon}>🤖</div>
                <h1>Join the Future</h1>
                <p className={styles.subtitle}>Start your Physical AI & Robotics journey</p>
              </div>

              {error && (
                <div className={styles.errorAlert} role="alert">
                  {error}
                </div>
              )}

              <form onSubmit={handleAccountSubmit} className={styles.authForm}>
                <div className={styles.formGroup}>
                  <label htmlFor="name">Full Name</label>
                  <input
                    type="text"
                    id="name"
                    name="name"
                    value={formData.name}
                    onChange={handleChange}
                    placeholder="John Doe"
                    required
                    className={styles.input}
                  />
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="email">Email Address</label>
                  <input
                    type="email"
                    id="email"
                    name="email"
                    value={formData.email}
                    onChange={handleChange}
                    placeholder="your.email@example.com"
                    required
                    className={styles.input}
                  />
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="password">Password</label>
                  <input
                    type="password"
                    id="password"
                    name="password"
                    value={formData.password}
                    onChange={handleChange}
                    placeholder="Create a strong password"
                    required
                    minLength={8}
                    className={styles.input}
                  />
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="confirmPassword">Confirm Password</label>
                  <input
                    type="password"
                    id="confirmPassword"
                    name="confirmPassword"
                    value={formData.confirmPassword}
                    onChange={handleChange}
                    placeholder="Confirm your password"
                    required
                    minLength={8}
                    className={styles.input}
                  />
                </div>

                <button type="submit" className={styles.submitButton}>
                  Next: Select Interests →
                </button>
              </form>

              <div className={styles.divider}>
                <span>or</span>
              </div>

              <div className={styles.alternativeAuth}>
                <p className={styles.authSwitch}>
                  Already have an account?{' '}
                  <a href="/signin" className={styles.link}>
                    Sign in
                  </a>
                </p>
              </div>

              <div className={styles.guestAccess}>
                <a href="/docs/intro" className={styles.guestLink}>
                  Continue as guest →
                </a>
              </div>
            </>
          )}

          {/* Step 2: Interests Selection */}
          {currentStep === 'interests' && (
            <>
              <div className={styles.authHeader}>
                <div className={styles.robotIcon}>🎯</div>
                <h1>Personalize Your Experience</h1>
                <p className={styles.subtitle}>Tell us about your background and interests</p>
              </div>

              {error && (
                <div className={styles.errorAlert} role="alert">
                  {error}
                </div>
              )}

              <div className={styles.interestsForm}>
                {/* Background Selection */}
                <div className={styles.formGroup}>
                  <label htmlFor="background">I am a...</label>
                  <select
                    id="background"
                    name="background"
                    value={formData.background}
                    onChange={handleChange}
                    className={styles.select}
                  >
                    <option value="student">Student</option>
                    <option value="professional">Professional</option>
                  </select>
                </div>

                {/* Language Preference */}
                <div className={styles.formGroup}>
                  <label htmlFor="languagePreference">Preferred Language</label>
                  <select
                    id="languagePreference"
                    name="languagePreference"
                    value={formData.languagePreference}
                    onChange={handleChange}
                    className={styles.select}
                  >
                    <option value="en">English</option>
                    <option value="ur">Urdu (اردو)</option>
                  </select>
                </div>

                {/* Interest Selector */}
                <InterestSelector
                  initialSelectedIds={selectedInterests}
                  onSelectionChange={setSelectedInterests}
                  showCounter={true}
                  allowSkip={false}
                />

                {/* Action Buttons */}
                <div className={styles.formActions}>
                  <button
                    type="button"
                    onClick={handleBackToAccount}
                    className={styles.secondaryButton}
                    disabled={isSubmitting}
                  >
                    ← Back
                  </button>
                  <button
                    type="button"
                    onClick={handleCompleteSignup}
                    className={styles.submitButton}
                    disabled={!isInterestsValid || isSubmitting}
                  >
                    {isSubmitting ? 'Creating Account...' : 'Complete Sign Up'}
                  </button>
                </div>
              </div>
            </>
          )}
        </div>
      </div>
    </Layout>
  );
}
