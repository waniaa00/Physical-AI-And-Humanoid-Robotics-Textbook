/**
 * Message List Component
 * Displays chat messages with auto-scroll and translation support
 */
import React, { useEffect, useRef, useState } from 'react';
import type { ChatMessage } from '../../../types/chat';
import { TranslateButton, TranslatedText } from '../Translation';
import { SummarizeButton, SummaryDisplay } from '../Summarization';
import '@site/src/css/translation.css';

interface MessageListProps {
  messages: ChatMessage[];
  isLoading: boolean;
}

interface TranslatedMessage {
  messageId: string;
  translatedText: string;
  originalText: string;
}

interface SummarizedMessage {
  messageId: string;
  summary: string;
  originalText: string;
  metrics: {
    original_word_count: number;
    summary_word_count: number;
    compression_ratio: number;
    processing_time_ms: number;
    key_points?: string[];
  };
}

export function MessageList({ messages, isLoading }: MessageListProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const [copiedId, setCopiedId] = useState<string | null>(null);
  const [translatedMessages, setTranslatedMessages] = useState<Map<string, TranslatedMessage>>(new Map());
  const [summarizedMessages, setSummarizedMessages] = useState<Map<string, SummarizedMessage>>(new Map());

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  const handleCopy = async (text: string, messageId: string) => {
    try {
      await navigator.clipboard.writeText(text);
      setCopiedId(messageId);
      setTimeout(() => setCopiedId(null), 2000);
    } catch (err) {
      console.error('Failed to copy text:', err);
    }
  };

  const handleTranslate = (messageId: string) => (translatedText: string, originalText: string) => {
    setTranslatedMessages(new Map(translatedMessages.set(messageId, {
      messageId,
      translatedText,
      originalText
    })));
  };

  const handleSummarize = (messageId: string) => (
    summary: string,
    originalText: string,
    metrics: SummarizedMessage['metrics']
  ) => {
    setSummarizedMessages(new Map(summarizedMessages.set(messageId, {
      messageId,
      summary,
      originalText,
      metrics
    })));
  };

  const isMessageTranslated = (messageId: string) => translatedMessages.has(messageId);
  const isMessageSummarized = (messageId: string) => summarizedMessages.has(messageId);

  return (
    <div className="h-full overflow-y-auto px-3 py-4 space-y-0" style={{
      scrollbarWidth: 'thin',
      scrollbarColor: '#93c5fd #f8fafc',
      backgroundColor: '#f8fafc',
      minHeight: '100%',
      paddingTop: '12px',
      paddingBottom: '12px'
    }}>
      {messages.length === 0 && !isLoading && (
        <div className="text-center mt-8" style={{ color: '#2563eb' }}>
          <div style={{ fontSize: '40px', marginBottom: '12px' }}>🤖</div>
          <p className="text-base font-semibold mb-1.5" style={{ color: '#1e40af', fontSize: '16px' }}>Robotics Assistant Ready</p>
          <p className="text-xs" style={{ color: '#64748b', lineHeight: '1.5', fontSize: '13px' }}>
            Ask me about humanoid robotics,<br />
            ROS 2, kinematics, or physical AI
          </p>
        </div>
      )}

      {messages.map((message) => (
        <div
          key={message.id}
          className={`flex ${message.role === 'user' ? 'justify-end' : 'justify-start'}`}
          style={{ marginBottom: '12px' }}
        >
          <div
            className="max-w-[85%] relative group"
            style={{
              backgroundColor: message.role === 'user' ? '#2563eb' : '#ffffff',
              color: message.role === 'user' ? '#ffffff' : '#1e293b',
              border: message.role === 'user' ? 'none' : '1px solid #e2e8f0',
              boxShadow: message.role === 'user'
                ? '0 4px 14px rgba(37, 99, 235, 0.35)'
                : '0 2px 10px rgba(0, 0, 0, 0.06)',
              borderRadius: '18px',
              wordWrap: 'break-word',
              wordBreak: 'break-word',
              overflowWrap: 'break-word',
              position: 'relative',
              padding: '14px 16px',
              minWidth: '60px',
              maxWidth: '100%'
            }}
          >
            {/* Copy, Translate, and Summarize buttons for AI messages */}
            {message.role === 'assistant' && (
              <div className="absolute top-2 right-2 opacity-0 group-hover:opacity-100 transition-all duration-200 flex gap-2 flex-wrap" style={{
                zIndex: 10,
                background: 'linear-gradient(135deg, rgba(255, 255, 255, 0.98) 0%, rgba(249, 250, 251, 0.98) 100%)',
                borderRadius: '14px',
                padding: '6px',
                boxShadow: '0 4px 16px rgba(0, 0, 0, 0.12), 0 2px 4px rgba(0, 0, 0, 0.06)',
                border: '1px solid rgba(226, 232, 240, 0.8)',
                backdropFilter: 'blur(8px)'
              }}>
                <button
                  onClick={() => handleCopy(message.content, message.id)}
                  aria-label="Copy message"
                  title={copiedId === message.id ? 'Copied!' : 'Copy text'}
                  className="hover-lift"
                  style={{
                    background: copiedId === message.id
                      ? 'linear-gradient(135deg, #10b981 0%, #059669 100%)'
                      : 'linear-gradient(135deg, #f8fafc 0%, #f1f5f9 100%)',
                    color: copiedId === message.id ? '#ffffff' : '#475569',
                    border: copiedId === message.id ? 'none' : '1px solid #e2e8f0',
                    borderRadius: '10px',
                    padding: '6px 10px',
                    cursor: 'pointer',
                    fontSize: '11px',
                    fontWeight: 600,
                    display: 'flex',
                    alignItems: 'center',
                    gap: '4px',
                    transition: 'all 0.2s ease',
                    height: '30px',
                    minWidth: '60px',
                    boxShadow: copiedId === message.id
                      ? '0 2px 8px rgba(16, 185, 129, 0.3)'
                      : '0 1px 3px rgba(0, 0, 0, 0.08)',
                    transform: 'scale(1)',
                  }}
                  onMouseEnter={(e) => {
                    if (copiedId !== message.id) {
                      e.currentTarget.style.transform = 'translateY(-1px) scale(1.02)';
                      e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.15)';
                      e.currentTarget.style.borderColor = '#cbd5e1';
                    }
                  }}
                  onMouseLeave={(e) => {
                    e.currentTarget.style.transform = 'scale(1)';
                    e.currentTarget.style.boxShadow = copiedId === message.id
                      ? '0 2px 8px rgba(16, 185, 129, 0.3)'
                      : '0 1px 3px rgba(0, 0, 0, 0.08)';
                    if (copiedId !== message.id) {
                      e.currentTarget.style.borderColor = '#e2e8f0';
                    }
                  }}
                >
                  {copiedId === message.id ? (
                    <>
                      <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="3" strokeLinecap="round">
                        <polyline points="20 6 9 17 4 12" />
                      </svg>
                      Copied
                    </>
                  ) : (
                    <>
                      <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round">
                        <rect x="9" y="9" width="13" height="13" rx="2" ry="2" />
                        <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1" />
                      </svg>
                      Copy
                    </>
                  )}
                </button>
                <div style={{ width: '105px', height: '30px' }}>
                  <TranslateButton
                    text={message.content}
                    onTranslate={handleTranslate(message.id)}
                    preserveCode={true}
                    style={{
                      height: '30px',
                      fontSize: '11px',
                      padding: '6px 10px',
                      background: 'linear-gradient(135deg, #3b82f6 0%, #2563eb 100%)',
                      border: 'none',
                      borderRadius: '10px',
                      color: '#ffffff',
                      fontWeight: 600,
                      boxShadow: '0 2px 8px rgba(59, 130, 246, 0.25)',
                      transition: 'all 0.2s ease',
                      cursor: 'pointer'
                    }}
                  />
                </div>
                <div style={{ width: '110px', height: '30px' }}>
                  <SummarizeButton
                    text={message.content}
                    onSummarize={handleSummarize(message.id)}
                    targetLength="short"
                    focus="concepts"
                    style={{
                      height: '30px',
                      fontSize: '11px',
                      padding: '6px 10px',
                      background: 'linear-gradient(135deg, #8b5cf6 0%, #7c3aed 100%)',
                      border: 'none',
                      borderRadius: '10px',
                      color: '#ffffff',
                      fontWeight: 600,
                      boxShadow: '0 2px 8px rgba(139, 92, 246, 0.25)',
                      transition: 'all 0.2s ease',
                      cursor: 'pointer'
                    }}
                  />
                </div>
              </div>
            )}

            {/* Message content: summarized, translated, or original */}
            <div style={{
              paddingRight: message.role === 'assistant' && message.citations && message.citations.length > 0 ? '0' : '0',
              marginBottom: '6px'
            }}>
              {isMessageSummarized(message.id) ? (
                <SummaryDisplay
                  summary={summarizedMessages.get(message.id)!.summary}
                  originalText={summarizedMessages.get(message.id)!.originalText}
                  metrics={summarizedMessages.get(message.id)!.metrics}
                  defaultShowSummary={true}
                />
              ) : isMessageTranslated(message.id) ? (
                <TranslatedText
                  originalText={translatedMessages.get(message.id)!.originalText}
                  translatedText={translatedMessages.get(message.id)!.translatedText}
                  defaultShowTranslated={true}
                />
              ) : (
                <div className="text-sm whitespace-pre-wrap break-words" style={{
                  lineHeight: '1.55',
                  fontFamily: 'system-ui, -apple-system, sans-serif',
                  fontSize: '14px',
                  wordWrap: 'break-word',
                  wordBreak: 'break-word',
                  overflowWrap: 'break-word',
                  minWidth: '0',
                  maxWidth: '100%',
                  color: message.role === 'user' ? '#f8fafc' : '#1e293b',
                  marginTop: '2px'
                }}>
                  {message.content}
                </div>
              )}
            </div>

            {message.citations && message.citations.length > 0 && (
              <div className="mt-2 pt-2 space-y-1.5" style={{
                borderTop: '1px solid #e0e7ff',
                marginTop: '8px',
                backgroundColor: '#f8fafc',
                borderRadius: '6px',
                padding: '6px 10px'
              }}>
                <div className="text-xs font-bold flex items-center gap-1" style={{
                  color: '#1e40af',
                  letterSpacing: '0.5px',
                  marginBottom: '4px'
                }}>
                  <svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71" />
                    <path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71" />
                  </svg>
                  SOURCES
                </div>
                {message.citations.map((citation) => (
                  <a
                    key={citation.number}
                    href={citation.url}
                    target="_blank"
                    rel="noopener noreferrer"
                    className="block text-xs hover:bg-blue-100 break-all"
                    style={{
                      color: '#1e40af',
                      padding: '6px 8px',
                      backgroundColor: '#f1f5f9',
                      borderRadius: '6px',
                      border: '1px solid #cbd5e1',
                      transition: 'all 0.2s',
                      textDecoration: 'none',
                      fontSize: '11px',
                      wordBreak: 'break-all',
                      overflowWrap: 'break-word'
                    }}
                  >
                    <span style={{ fontWeight: 'bold', color: '#2563eb' }}>[{citation.number}]</span>{' '}
                    <span style={{ opacity: 0.85, wordBreak: 'break-all', fontSize: '10px' }}>{citation.url}</span>
                  </a>
                ))}
              </div>
            )}
            <div className="text-xs mt-2 flex items-center justify-end gap-1.5" style={{
              opacity: 0.7,
              color: message.role === 'user' ? '#93c5fd' : '#64748b',
              marginTop: '6px',
              fontSize: '11px'
            }}>
              <svg width="9" height="9" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <circle cx="12" cy="12" r="10" />
                <polyline points="12 6 12 12 16 14" />
              </svg>
              {message.timestamp.toLocaleTimeString([], {
                hour: '2-digit',
                minute: '2-digit',
              })}
            </div>
          </div>
        </div>
      ))}

      {isLoading && (
        <div className="flex justify-start" style={{ marginBottom: '12px' }}>
          <div className="" style={{
            backgroundColor: '#ffffff',
            border: '1px solid #e2e8f0',
            boxShadow: '0 2px 10px rgba(0, 0, 0, 0.06)',
            borderRadius: '18px',
            padding: '14px 16px',
            minWidth: '60px',
            maxWidth: '85%'
          }}>
            <div className="flex items-center gap-3">
              <div className="flex space-x-1.5">
                <div className="w-2.5 h-2.5 rounded-full animate-bounce" style={{
                  backgroundColor: '#2563eb',
                  animationDelay: '0ms'
                }} />
                <div className="w-2.5 h-2.5 rounded-full animate-bounce" style={{
                  backgroundColor: '#2563eb',
                  animationDelay: '150ms'
                }} />
                <div className="w-2.5 h-2.5 rounded-full animate-bounce" style={{
                  backgroundColor: '#2563eb',
                  animationDelay: '300ms'
                }} />
              </div>
              <span className="text-xs font-medium" style={{ color: '#64748b', fontSize: '13px' }}>AI is thinking...</span>
            </div>
          </div>
        </div>
      )}

      <div ref={messagesEndRef} />
    </div>
  );
}
