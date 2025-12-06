import React, { useState } from 'react';
import Layout from '@theme/Layout';
import styles from './auth.module.css';

export default function SignIn() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    // TODO: Implement authentication
    console.log('Sign in:', { email, password });
  };

  return (
    <Layout title="Sign In" description="Sign in to access the Physical AI & Humanoid Robotics course">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.authHeader}>
            <div className={styles.robotIcon}>🤖</div>
            <h1>Welcome Back</h1>
            <p className={styles.subtitle}>Sign in to continue your robotics journey</p>
          </div>

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

            <button type="submit" className={styles.submitButton}>
              Sign In
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
        </div>
      </div>
    </Layout>
  );
}
