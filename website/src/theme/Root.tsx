/**
 * Root Component
 * Wraps the entire Docusaurus app to add global functionality
 */
import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { AuthProvider } from '../contexts/AuthContext';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      {children}
      <BrowserOnly fallback={<div />}>
        {() => {
          console.log('[ChatKit] Rendering ChatKit component...');
          const { ChatKit } = require('../components/ChatKit');
          console.log('[ChatKit] ChatKit imported:', ChatKit);
          return <ChatKit />;
        }}
      </BrowserOnly>
    </AuthProvider>
  );
}
