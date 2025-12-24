/**
 * Profile Page (T039)
 * Allows users to view and update their interests, background, and language preference.
 * Triggers session refresh when interests are updated to enable personalized content updates.
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { InterestSelector } from '@site/src/components/Interests';
import { setUserHasInterests } from '../lib/user';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './auth.module.css';

interface UserProfile {
  user_id: string;
  background: 'student' | 'professional';
  language_preference: 'en' | 'ur';
  interests: Array<{
    id: number;
    name: string;
    slug: string;
    description: string;
  }>;
  created_at: string;
  updated_at: string;
}

export default function Profile() {
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isEditing, setIsEditing] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  // Form state for editing
  const [editFormData, setEditFormData] = useState({
    background: 'student' as 'student' | 'professional',
    languagePreference: 'en' as 'en' | 'ur',
  });
  const [selectedInterests, setSelectedInterests] = useState<number[]>([]);

  // T039: Get auth context to trigger session refresh
  const { validateSession, signOut, user, isAuthenticated, isLoading: authLoading } = useAuth();

  useEffect(() => {
    // Wait for auth to load
    if (authLoading) return;

    // Check if user is signed in
    if (!isAuthenticated || !user) {
      window.location.href = '/signin';
      return;
    }

    fetchProfile();
  }, [isAuthenticated, user, authLoading]);

  const fetchProfile = async () => {
    if (!user?.user_id) return;

    setIsLoading(true);
    setError(null);

    try {
      const apiUrl = 'https://wnxddev-humanoid-robotics-api.hf.space';
      const response = await fetch(`${apiUrl}/interests/${user.user_id}`);

      if (response.status === 404) {
        // User has no interests yet
        setProfile(null);
        setIsEditing(true); // Start in edit mode
        setIsLoading(false);
        return;
      }

      if (!response.ok) {
        throw new Error('Failed to fetch profile');
      }

      const data: UserProfile = await response.json();
      setProfile(data);

      // Initialize edit form with current data
      setEditFormData({
        background: data.background,
        languagePreference: data.language_preference,
      });
      setSelectedInterests(data.interests.map((i) => i.id));
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to load profile';
      setError(errorMessage);
      console.error('Profile fetch error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const handleStartEdit = () => {
    setIsEditing(true);
    setSuccessMessage(null);
  };

  const handleCancelEdit = () => {
    if (profile) {
      setIsEditing(false);
      // Reset form to current profile data
      setEditFormData({
        background: profile.background,
        languagePreference: profile.language_preference,
      });
      setSelectedInterests(profile.interests.map((i) => i.id));
    } else {
      // No profile exists, redirect to signup
      window.location.href = '/signup';
    }
  };

  const handleSaveProfile = async () => {
    setIsSaving(true);
    setError(null);
    setSuccessMessage(null);

    try {
      if (!user?.user_id) {
        throw new Error('User not authenticated');
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
          background: editFormData.background,
          language_preference: editFormData.languagePreference,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Failed to save profile');
      }

      // Mark user as having interests for personalization
      setUserHasInterests(true);

      // T039: Validate session to update AuthContext with new interests
      // This ensures the personalized content page will see the updated interests
      await validateSession();

      // Refresh profile
      await fetchProfile();
      setIsEditing(false);
      setSuccessMessage('Profile updated successfully! Your personalized content will update automatically.');

      // Clear success message after 3 seconds
      setTimeout(() => setSuccessMessage(null), 3000);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to save profile';
      setError(errorMessage);
      console.error('Profile save error:', err);
    } finally {
      setIsSaving(false);
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setEditFormData({
      ...editFormData,
      [e.target.name]: e.target.value,
    });
  };

  const isInterestsValid = selectedInterests.length >= 2 && selectedInterests.length <= 5;

  return (
    <Layout title="Profile" description="Manage your learning profile and interests">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.authHeader}>
            <div className={styles.robotIcon}>👤</div>
            <h1>My Profile</h1>
            <p className={styles.subtitle}>
              Manage your interests and preferences
            </p>
          </div>

          {/* Sign Out Button */}
          <div style={{ padding: '0 1.5rem', marginBottom: '1.5rem' }}>
            <button
              onClick={async () => {
                try {
                  await signOut();
                  window.location.href = '/';
                } catch (error) {
                  console.error('Sign out error:', error);
                }
              }}
              style={{
                width: '100%',
                padding: '0.75rem',
                backgroundColor: '#ef4444',
                color: 'white',
                border: 'none',
                borderRadius: '8px',
                fontSize: '0.875rem',
                fontWeight: 500,
                cursor: 'pointer',
                transition: 'background 0.2s ease',
              }}
              onMouseOver={(e) => (e.currentTarget.style.backgroundColor = '#dc2626')}
              onMouseOut={(e) => (e.currentTarget.style.backgroundColor = '#ef4444')}
            >
              🚪 Sign Out
            </button>
          </div>

          {/* Loading State */}
          {isLoading && (
            <div style={{ textAlign: 'center', padding: '3rem' }}>
              <div
                style={{
                  display: 'inline-block',
                  width: '40px',
                  height: '40px',
                  border: '4px solid #e2e8f0',
                  borderTopColor: '#3b82f6',
                  borderRadius: '50%',
                  animation: 'spin 0.8s linear infinite',
                }}
              ></div>
              <p style={{ marginTop: '1rem', color: '#64748b' }}>Loading profile...</p>
            </div>
          )}

          {/* Success Message */}
          {successMessage && (
            <div
              style={{
                padding: '0.875rem 1rem',
                backgroundColor: '#d1fae5',
                border: '1px solid #a7f3d0',
                borderLeft: '4px solid #10b981',
                borderRadius: '8px',
                color: '#065f46',
                fontSize: '0.875rem',
                marginBottom: '1.5rem',
              }}
              role="alert"
            >
              {successMessage}
            </div>
          )}

          {/* Error Message */}
          {error && (
            <div className={styles.errorAlert} role="alert">
              {error}
            </div>
          )}

          {/* Profile View Mode */}
          {!isLoading && !isEditing && profile && (
            <div style={{ display: 'flex', flexDirection: 'column', gap: '1.5rem' }}>
              {/* Profile Info */}
              <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
                <div>
                  <div style={{ fontSize: '0.75rem', fontWeight: 600, color: '#64748b', textTransform: 'uppercase', marginBottom: '0.25rem' }}>
                    Background
                  </div>
                  <div style={{ fontSize: '1rem', fontWeight: 500, color: '#1e293b' }}>
                    {profile.background === 'student' ? '🎓 Student' : '💼 Professional'}
                  </div>
                </div>

                <div>
                  <div style={{ fontSize: '0.75rem', fontWeight: 600, color: '#64748b', textTransform: 'uppercase', marginBottom: '0.25rem' }}>
                    Language Preference
                  </div>
                  <div style={{ fontSize: '1rem', fontWeight: 500, color: '#1e293b' }}>
                    {profile.language_preference === 'en' ? '🇬🇧 English' : '🇵🇰 Urdu (اردو)'}
                  </div>
                </div>

                <div>
                  <div style={{ fontSize: '0.75rem', fontWeight: 600, color: '#64748b', textTransform: 'uppercase', marginBottom: '0.25rem' }}>
                    Your Interests ({profile.interests.length})
                  </div>
                  <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem', marginTop: '0.5rem' }}>
                    {profile.interests.map((interest) => (
                      <span
                        key={interest.id}
                        style={{
                          padding: '0.5rem 1rem',
                          backgroundColor: '#eff6ff',
                          color: '#1e40af',
                          borderRadius: '9999px',
                          fontSize: '0.875rem',
                          fontWeight: 500,
                          border: '2px solid #dbeafe',
                        }}
                      >
                        {interest.name}
                      </span>
                    ))}
                  </div>
                </div>

                <div style={{ fontSize: '0.75rem', color: '#94a3b8', marginTop: '0.5rem' }}>
                  Last updated: {new Date(profile.updated_at).toLocaleDateString()}
                </div>
              </div>

              {/* Edit Button */}
              <button onClick={handleStartEdit} className={styles.submitButton}>
                Edit Profile
              </button>
            </div>
          )}

          {/* Profile Edit Mode */}
          {!isLoading && isEditing && (
            <div className={styles.interestsForm}>
              {/* Background Selection */}
              <div className={styles.formGroup}>
                <label htmlFor="background">I am a...</label>
                <select
                  id="background"
                  name="background"
                  value={editFormData.background}
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
                  value={editFormData.languagePreference}
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
                  onClick={handleCancelEdit}
                  className={styles.secondaryButton}
                  disabled={isSaving}
                >
                  Cancel
                </button>
                <button
                  type="button"
                  onClick={handleSaveProfile}
                  className={styles.submitButton}
                  disabled={!isInterestsValid || isSaving}
                >
                  {isSaving ? 'Saving...' : 'Save Changes'}
                </button>
              </div>
            </div>
          )}
        </div>
      </div>
    </Layout>
  );
}
