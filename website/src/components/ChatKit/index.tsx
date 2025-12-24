/**
 * ChatKit - Complete chat interface
 * Exports main chat component with button and window
 */
export { ChatButton } from './ChatButton';
export { ChatWindow } from './ChatWindow';
export { MessageList } from './MessageList';
export { ChatInput } from './ChatInput';

// Main export
import React, { useState } from 'react';
import { ChatButton } from './ChatButton';
import { ChatWindow } from './ChatWindow';
import { ChatProvider } from '../../contexts/ChatContext';

export function ChatKit() {
  const [isOpen, setIsOpen] = useState(false);

  console.log('[ChatKit] ChatKit component rendering, isOpen:', isOpen);

  return (
    <ChatProvider>
      <div data-chatkit="root" style={{ position: 'fixed', zIndex: 9999 }}>
        <ChatButton isOpen={isOpen} onClick={() => {
          console.log('[ChatKit] Button clicked, toggling from:', isOpen);
          setIsOpen(!isOpen);
        }} />
        <ChatWindow isOpen={isOpen} onClose={() => setIsOpen(false)} />
      </div>
    </ChatProvider>
  );
}
