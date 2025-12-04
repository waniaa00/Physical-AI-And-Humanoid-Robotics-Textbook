import React from 'react';
import Layout from '@theme/Layout';
import styles from './auth.module.css';

export default function SignUp() {
  return (
    <Layout title="Sign Up">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>Create Account</h1>
          <p style={{ textAlign: 'center', color: '#6b7280', marginBottom: '2rem' }}>
            Authentication is coming soon! This course is currently in development.
          </p>
          <div style={{ textAlign: 'center' }}>
            <a href="/docs/intro" className="button button--primary button--lg">
              Browse Course Content
            </a>
          </div>
        </div>
      </div>
    </Layout>
  );
}
