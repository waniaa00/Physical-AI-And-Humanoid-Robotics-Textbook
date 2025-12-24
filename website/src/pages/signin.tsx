import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { InterestSelector } from '@site/src/components/Interests';
import { setCurrentUserId, setUserHasInterests } from '../lib/user';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './auth.module.css';

type SignInStep = 'credentials' | 'interests';

export default function SignIn() {
  const { signIn, user, refreshUserInterests } = useAuth(); // T041: Integrate AuthContext - get user object and refresh method
  const [currentStep, setCurrentStep] = useState<SignInStep>('credentials');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showInterestsPrompt, setShowInterestsPrompt] = useState(false);
  const [selectedInterests, setSelectedInterests] = useState<number[]>([]);
  const [formData, setFormData] = useState({
    background: 'student' as 'student' | 'professional',
    languagePreference: 'en' as 'en' | 'ur',
  });
  const [isSaving, setIsSaving] = useState(false);
  const [isSigningIn, setIsSigningIn] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value,
    });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsSigningIn(true);

    try {
      // T041: Use AuthContext signIn method
      await signIn({ email, password });

      // Wait a bit for auth context to update
      await new Promise(resolve => setTimeout(resolve, 500));

      console.log('[SignIn] Sign in successful');

      // Check for returnTo parameter in URL
      const urlParams = new URLSearchParams(window.location.search);
      const returnTo = urlParams.get('returnTo');

      // Check if user has interests (user object is now available from auth context)
      // Note: we need to call validateSession or check the updated user state
      // For now, redirect to personalized content which will handle the prompt
      window.location.href = returnTo || '/personalized-content';
    } catch (err) {
      // T042: Display error message for invalid credentials
      const errorMessage = err instanceof Error ? err.message : 'Sign in failed';
      setError(errorMessage);
      console.error('Sign in error:', err);
    } finally {
      setIsSigningIn(false);
    }
  };

  const handleAddInterests = () => {
    setShowInterestsPrompt(false);
    setCurrentStep('interests');
  };

  const handleSkipInterests = () => {
    // Skip interests and go to personalized content page
    window.location.href = '/personalized-content';
  };

  const handleSaveInterests = async () => {
    setIsSaving(true);
    setError(null);

    try {
      // Use actual user ID from auth context
      if (!user?.user_id) {
        throw new Error('User not authenticated. Please sign in first.');
      }

      const apiUrl = 'https://wnxddev-humanoid-robotics-api.hf.space';

      const response = await fetch(`${apiUrl}/interests/save`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id: user.user_id,
          interest_ids: selectedInterests,
          background: formData.background,
          language_preference: formData.languagePreference,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Failed to save interests');
      }

      // Mark user as having interests for personalization
      setUserHasInterests(true);

      // Refresh interests in auth context so personalized content page has them
      await refreshUserInterests();

      // Success! Redirect to personalized content page
      console.log('[SignIn] Interests saved, redirecting to personalized content');
      window.location.href = '/personalized-content';
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to save interests';
      setError(errorMessage);
      console.error('Save interests error:', err);
    } finally {
      setIsSaving(false);
    }
  };

  const isInterestsValid = selectedInterests.length >= 2 && selectedInterests.length <= 5;

  return (
    <Layout title="Sign In" description="Sign in to access the Physical AI & Humanoid Robotics course">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          {/* Step 1: Sign In Credentials */}
          {currentStep === 'credentials' && !showInterestsPrompt && (
            <>
              <div className={styles.authHeader}>
                <div className={styles.robotIcon}>🤖</div>
                <h1>Welcome Back</h1>
                <p className={styles.subtitle}>Sign in to continue your robotics journey</p>
              </div>

              {error && (
                <div className={styles.errorAlert} role="alert">
                  {error}
                </div>
              )}

              <form onSubmit={handleSubmit} className={styles.authForm}>
                <div className={styles.formGroup}>
                  <label htmlFor="email">Email Address</label>
                  <input
                    type="email"
                    id="email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
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
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    placeholder="Enter your password"
                    required
                    className={styles.input}
                  />
                </div>

                <div className={styles.formFooter}>
                  <a href="/forgot-password" className={styles.link}>
                    Forgot password?
                  </a>
                </div>

                <button type="submit" className={styles.submitButton} disabled={isSigningIn}>
                  {isSigningIn ? 'Signing In...' : 'Sign In'}
                </button>
              </form>

              <div className={styles.divider}>
                <span>or</span>
              </div>

              <div className={styles.alternativeAuth}>
                <p className={styles.authSwitch}>
                  Don't have an account?{' '}
                  <a href="/signup" className={styles.link}>
                    Sign up
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

          {/* Interests Prompt (Optional Step) */}
          {currentStep === 'credentials' && showInterestsPrompt && (
            <>
              <div className={styles.authHeader}>
                <div className={styles.robotIcon}>🎯</div>
                <h1>One More Thing...</h1>
                <p className={styles.subtitle}>Help us personalize your learning experience</p>
              </div>

              <div style={{ padding: '2rem 1rem', textAlign: 'center' }}>
                <p style={{ fontSize: '1rem', color: '#475569', marginBottom: '2rem', lineHeight: '1.6' }}>
                  We noticed you haven't selected your interests yet. Taking a moment to tell us what you're interested in will help us provide more relevant examples and explanations.
                </p>

                <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
                  <button onClick={handleAddInterests} className={styles.submitButton}>
                    Add My Interests
                  </button>
                  <button onClick={handleSkipInterests} className={styles.secondaryButton}>
                    Skip for Now
                  </button>
                </div>
              </div>
            </>
          )}

          {/* Step 2: Add Interests (Optional) */}
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
                    onClick={handleSkipInterests}
                    className={styles.secondaryButton}
                    disabled={isSaving}
                  >
                    Skip
                  </button>
                  <button
                    type="button"
                    onClick={handleSaveInterests}
                    className={styles.submitButton}
                    disabled={!isInterestsValid || isSaving}
                  >
                    {isSaving ? 'Saving...' : 'Save & Continue'}
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
