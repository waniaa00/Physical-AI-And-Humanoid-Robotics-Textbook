/**
 * Authentication Context for Better Auth Integration (T026).
 *
 * Provides:
 * - User authentication state
 * - Sign-up, sign-in, sign-out methods
 * - Session persistence
 * - Automatic session validation on app load (T056)
 */

import React, { createContext, useContext, useState, useEffect } from 'react';
import axios from 'axios';
import Cookies from 'js-cookie';

// Use localhost for development - Docusaurus runs in browser, not Node
const API_BASE_URL = 'http://localhost:8000';

interface User {
  user_id: string;
  email: string;
  interests: number[];
}

interface SignUpData {
  email: string;
  password: string;
  interests: number[];
  background: 'student' | 'professional';
  language_preference: 'en' | 'ur';
}

interface SignInData {
  email: string;
  password: string;
}

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  signUp: (data: SignUpData) => Promise<void>;
  signIn: (data: SignInData) => Promise<void>;
  signOut: () => Promise<void>;
  validateSession: () => Promise<boolean>;
  refreshUserInterests: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true); // Start as true to wait for session validation

  // T056: Automatic session validation on app load
  useEffect(() => {
    const initAuth = async () => {
      try {
        console.log('[AuthContext] Validating session...');
        await validateSession();
      } catch (error) {
        console.error('[AuthContext] Session validation error:', error);
        setIsLoading(false);
        setUser(null);
      }
    };
    initAuth();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  /**
   * Sign up new user (T026).
   */
  const signUp = async (data: SignUpData): Promise<void> => {
    try {
      const response = await axios.post(`${API_BASE_URL}/auth/sign-up`, data, {
        withCredentials: true, // Include cookies
      });

      setUser({
        user_id: response.data.user_id,
        email: data.email,
        interests: data.interests,
      });
    } catch (error: any) {
      console.error('Sign-up error:', error);
      throw new Error(error.response?.data?.detail || 'Sign-up failed');
    }
  };

  /**
   * Sign in existing user (T041, T042).
   */
  const signIn = async (data: SignInData): Promise<void> => {
    try {
      console.log('[AuthContext] Signing in...');
      const response = await axios.post(`${API_BASE_URL}/auth/sign-in`, data, {
        withCredentials: true, // Include cookies
      });

      console.log('[AuthContext] Sign-in successful, user_id:', response.data.user_id);
      setUser({
        user_id: response.data.user_id,
        email: data.email,
        interests: response.data.interests,
      });
    } catch (error: any) {
      console.error('Sign-in error:', error);
      throw new Error(error.response?.data?.detail || 'Sign-in failed');
    }
  };

  /**
   * Sign out user (T054).
   */
  const signOut = async (): Promise<void> => {
    try {
      await axios.post(`${API_BASE_URL}/auth/sign-out`, {}, {
        withCredentials: true,
      });

      setUser(null);
      Cookies.remove('session_token');
    } catch (error: any) {
      console.error('Sign-out error:', error);
      // Clear user state even if API call fails
      setUser(null);
      Cookies.remove('session_token');
    }
  };

  /**
   * Validate session token (T056, T044, T039).
   * Also fetches user interests to support personalized content updates.
   */
  const validateSession = async (): Promise<boolean> => {
    try {
      const response = await axios.get(`${API_BASE_URL}/auth/session/validate`, {
        withCredentials: true,
      });

      if (response.data.valid) {
        // T039: Fetch user interests to keep AuthContext up-to-date
        let userInterests: number[] = [];
        try {
          const interestsResponse = await axios.get(
            `${API_BASE_URL}/interests/${response.data.user_id}`
          );
          if (interestsResponse.status === 200 && interestsResponse.data.interests) {
            userInterests = interestsResponse.data.interests.map((i: any) => i.id);
          }
        } catch (err) {
          // User may not have interests yet - that's okay
          console.log('[AuthContext] User has no interests yet');
        }

        setUser({
          user_id: response.data.user_id,
          email: response.data.email,
          interests: userInterests,
        });
        return true;
      }
    } catch (error) {
      // Session invalid or expired - this is expected for logged-out users
      setUser(null);
    } finally {
      setIsLoading(false);
    }

    return false;
  };

  /**
   * Refresh user interests after they are updated.
   * Useful after saving interests on signin/signup.
   */
  const refreshUserInterests = async (): Promise<void> => {
    if (!user?.user_id) {
      console.log('[AuthContext] Cannot refresh interests - no user');
      return;
    }

    try {
      console.log('[AuthContext] Refreshing user interests...');
      const interestsResponse = await axios.get(
        `${API_BASE_URL}/interests/${user.user_id}`
      );

      if (interestsResponse.status === 200 && interestsResponse.data.interests) {
        const userInterests = interestsResponse.data.interests.map((i: any) => i.id);
        setUser({
          ...user,
          interests: userInterests,
        });
        console.log('[AuthContext] Interests refreshed:', userInterests);
      }
    } catch (err) {
      console.error('[AuthContext] Failed to refresh interests:', err);
    }
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        isAuthenticated: user !== null,
        isLoading,
        signUp,
        signIn,
        signOut,
        validateSession,
        refreshUserInterests,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

/**
 * Custom hook to access auth context.
 * Throws error if used outside AuthProvider.
 */
export const useAuthContext = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuthContext must be used within an AuthProvider');
  }
  return context;
};
