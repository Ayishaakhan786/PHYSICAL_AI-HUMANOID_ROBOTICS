import React, { useState, useRef, useEffect } from 'react';
import './ChatInput.css';

interface ChatInputProps {
  onSendMessage: (query: string) => void;
  disabled: boolean;
}

/**
 * ChatInput - Input field and send button
 *
 * Allows users to type questions and submit them to the chatbot.
 */
function ChatInput({ onSendMessage, disabled }: ChatInputProps) {
  const [input, setInput] = useState<string>('');
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Auto-focus on mount
  useEffect(() => {
    if (inputRef.current && !disabled) {
      inputRef.current.focus();
    }
  }, [disabled]);

  /**
   * Handle form submission
   */
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (input.trim() && !disabled) {
      onSendMessage(input.trim());
      setInput('');
    }
  };

  /**
   * Handle Enter key (submit on Enter, allow Shift+Enter for newline)
   */
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <form className="chat-input" onSubmit={handleSubmit}>
      <textarea
        ref={inputRef}
        className="chat-input-field"
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder="Ask a question about the book..."
        disabled={disabled}
        rows={1}
        maxLength={1000}
      />
      <button
        type="submit"
        className="chat-input-send"
        disabled={disabled || !input.trim()}
        title="Send message"
      >
        {disabled ? '...' : '➤'}
      </button>
    </form>
  );
}

export default ChatInput;
