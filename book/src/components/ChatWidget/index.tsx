import React, { useState, useEffect, useCallback } from 'react';
import ChatMessages from './ChatMessages';
import ChatInput from './ChatInput';
import SourceCitation from './SourceCitation';
import './chatWidget.css';

/**
 * ChatWidget - Main chat interface component
 *
 * Provides embedded chat functionality for asking questions about book content.
 * Manages conversation state, API calls, and displays responses with citations.
 */
function ChatWidget() {
  // State management
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [collapsed, setCollapsed] = useState<boolean>(false);
  const [minimized, setMinimized] = useState<boolean>(false);

  // Initialize session ID on mount
  useEffect(() => {
    // Get or create session ID from sessionStorage
    let id = sessionStorage.getItem('chat_session_id');
    if (!id) {
      // Generate UUID v4 (simple implementation)
      id = 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
        const r = Math.random() * 16 | 0;
        const v = c === 'x' ? r : (r & 0x3 | 0x8);
        return v.toString(16);
      });
      sessionStorage.setItem('chat_session_id', id);
    }
    setSessionId(id);
    console.log(`ChatWidget initialized with session: ${id}`);
  }, []);

  /**
   * Send message to backend API
   */
  const sendMessage = useCallback(async (queryText: string, selectedText: string | null = null, sourceUrl: string | null = null) => {
    if (!queryText || queryText.trim() === '') {
      setError('Please enter a question');
      return;
    }

    if (!sessionId) {
      setError('Session not initialized');
      return;
    }

    setLoading(true);
    setError(null);

    // Add user message optimistically
    const userMessage: ChatMessage = {
      role: 'user',
      content: queryText,
      timestamp: new Date().toISOString(),
    };
    setMessages(prev => [...prev, userMessage]);

    try {
      // Prepare request body
      const requestBody = {
        query_text: queryText,
        session_id: sessionId,
        response_format: 'json', // Use JSON for MVP
      };

      // Add optional context
      if (selectedText) {
        (requestBody as any).selected_text = selectedText;
      }
      if (sourceUrl) {
        (requestBody as any).source_url = sourceUrl;
      }

      console.log('Sending chat request:', requestBody);

      // Call backend API
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      console.log('Chat response:', data);

      // Add assistant message with sources
      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: data.answer_text,
        sources: data.sources || [],
        confidence: data.confidence,
        timestamp: new Date().toISOString(),
        latency_ms: data.latency_ms,
      };
      setMessages(prev => [...prev, assistantMessage]);

    } catch (err) {
      console.error('Chat request failed:', err);
      setError((err as Error).message || 'Failed to send message. Please try again.');
      // Remove optimistic user message on error
      setMessages(prev => prev.slice(0, -1));
    } finally {
      setLoading(false);
    }
  }, [sessionId]);

  /**
   * Handle error dismissal
   */
  const dismissError = useCallback(() => {
    setError(null);
  }, []);

  /**
   * Handle collapse toggle
   */
  const toggleCollapse = useCallback(() => {
    setCollapsed(prev => !prev);
  }, []);

  /**
   * Handle minimize toggle
   */
  const toggleMinimize = useCallback(() => {
    setMinimized(prev => !prev);
  }, []);

  if (minimized) {
    return (
      <div className="chat-widget-container">
        <button
          className="chat-widget-minimized"
          onClick={toggleMinimize}
          title="Open chat"
        >
          💬
        </button>
      </div>
    );
  }

  return (
    <div className="chat-widget-container">
      <div className={`chat-widget ${collapsed ? 'collapsed' : ''}`}>
        {/* Header */}
        <div className="chat-widget-header">
        <div className="chat-widget-title">
          <h3>Ask about this book</h3>
        </div>
        <div className="chat-widget-controls">
          <button
            className="chat-widget-collapse-btn"
            onClick={toggleCollapse}
            title={collapsed ? 'Expand' : 'Collapse'}
          >
            {collapsed ? '⬆' : '⬇'}
          </button>
          <button
            className="chat-widget-minimize-btn"
            onClick={toggleMinimize}
            title="Minimize"
          >
            −
          </button>
        </div>
      </div>

      {/* Messages */}
      {!collapsed && (
        <ChatMessages
          messages={messages}
          loading={loading}
        />
      )}

      {/* Error */}
      {error && !collapsed && (
        <div className="chat-widget-error">
          <span className="chat-widget-error-message">{error}</span>
          <button
            className="chat-widget-error-dismiss"
            onClick={dismissError}
          >
            ×
          </button>
        </div>
      )}

      {/* Input */}
      {!collapsed && (
        <ChatInput
          onSendMessage={sendMessage}
          disabled={loading}
        />
      )}
      </div>
    </div>
  );
}

export default ChatWidget;

// Type definitions
interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  sources?: any[];
  confidence?: 'high' | 'medium' | 'low';
  timestamp?: string;
  latency_ms?: number;
}
