import React from 'react';
import SourceCitation from './SourceCitation';
import './ChatMessages.css';

/**
 * ChatMessages - Display chat history
 *
 * Shows user queries and assistant responses with source citations.
 */
interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  sources?: any[];
  confidence?: 'high' | 'medium' | 'low';
  timestamp?: string;
  latency_ms?: number;
}

interface ChatMessagesProps {
  messages: ChatMessage[];
  loading: boolean;
}

function ChatMessages({ messages, loading }: ChatMessagesProps) {
  const messagesEndRef = React.useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  React.useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  return (
    <div className="chat-messages">
      {messages.length === 0 && !loading && (
        <div className="chat-messages-empty">
          <p>Ask a question about Physical AI book content!</p>
          <p className="chat-messages-empty-hint">
            Examples:
          </p>
          <ul className="chat-messages-examples">
            <li>"What is embodied AI?"</li>
            <li>"How do VLA models work?"</li>
            <li>"Explain domain randomization"</li>
          </ul>
        </div>
      )}

      {messages.map((msg, index) => (
        <div
          key={index}
          className={`chat-message chat-message-${msg.role}`}
          ref={index === messages.length - 1 ? messagesEndRef : null}
        >
          {/* User message */}
          {msg.role === 'user' && (
            <div className="chat-message-content">
              <div className="chat-message-text">{msg.content}</div>
              <div className="chat-message-timestamp">
                {msg.timestamp ? new Date(msg.timestamp).toLocaleTimeString() : ''}
              </div>
            </div>
          )}

          {/* Assistant message */}
          {msg.role === 'assistant' && (
            <div className="chat-message-content">
              <div
                className="chat-message-text"
                dangerouslySetInnerHTML={{ __html: msg.content }}
              />

              {/* Confidence indicator */}
              {msg.confidence && (
                <div className={`chat-message-confidence chat-message-confidence-${msg.confidence}`}>
                  Confidence: {msg.confidence}
                </div>
              )}

              {/* Latency */}
              {msg.latency_ms && (
                <div className="chat-message-latency">
                  {msg.latency_ms}ms
                </div>
              )}

              {/* Source citations */}
              {msg.sources && msg.sources.length > 0 && (
                <div className="chat-message-sources">
                  <div className="chat-message-sources-title">
                    Sources:
                  </div>
                  {msg.sources.map((source: any, idx: number) => (
                    <SourceCitation
                      key={idx}
                      source={source}
                    />
                  ))}
                </div>
              )}

              {/* Timestamp */}
              <div className="chat-message-timestamp">
                {msg.timestamp ? new Date(msg.timestamp).toLocaleTimeString() : ''}
              </div>
            </div>
          )}
        </div>
      ))}

      {/* Loading indicator */}
      {loading && (
        <div className="chat-message chat-message-loading">
          <div className="chat-loading-indicator">
            <span className="chat-loading-dot"></span>
            <span className="chat-loading-dot"></span>
            <span className="chat-loading-dot"></span>
          </div>
          <div className="chat-loading-text">Thinking...</div>
        </div>
      )}
    </div>
  );
}

export default ChatMessages;
