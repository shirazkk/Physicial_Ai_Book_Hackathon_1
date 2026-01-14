import React, { useState, useRef, useEffect } from 'react';
import chatApiService from '../../services/chat-api';
import TextSelector from './TextSelector';

import './ChatWidget.css'; // Import the CSS file

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedTextOnlyMode, setSelectedTextOnlyMode] = useState(false);
  const [selectedText, setSelectedText] = useState('');

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle sending a message
  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      // Send the question to the backend
      const response = await chatApiService.askQuestion(
        currentInput,
        sessionId,
        selectedTextOnlyMode,
        selectedTextOnlyMode ? selectedText : null
      );

      // Update session ID if new one was created
      if (response.session_id && !sessionId) {
        setSessionId(response.session_id);
      }

      // Add bot response to the chat
      const botMessage = {
        id: Date.now() + 1,
        text: response.response || response.answer || 'Sorry, I could not generate a response.',
        sender: 'bot',
        timestamp: new Date().toISOString(),
        citations: response.citations || []
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error getting chat response:', error);

      // Add error message to the chat
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, there was an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date().toISOString(),
        isError: true
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle key press (Enter to send)
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Toggle chat widget open/close
  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  // Clear chat history
  const clearChat = () => {
    setMessages([]);
    setSessionId(null);
  };

  // Toggle selected text only mode
  const toggleSelectedTextMode = () => {
    setSelectedTextOnlyMode(!selectedTextOnlyMode);
  };

  // Callback function to handle text selection from TextSelector
  const handleTextSelection = (selectedTextContent) => {
    if (selectedTextContent) {
      setSelectedText(selectedTextContent);
    }
  };

  return (
    <div className="chat-widget">
      {/* TextSelector component to capture text selections across the page */}
      <TextSelector onTextSelected={handleTextSelection} />

      {/* Floating button to open/close chat */}
      {!isOpen && (
        <button className="chat-toggle-btn" onClick={toggleChat}>
          💬
        </button>
      )}

      {/* Chat container */}
      {isOpen && (
        <div className="chat-container">
          <div className="chat-header">
            <div className="chat-header-left">
              <h3>📚 Book Assistant</h3>
              <div className="chat-controls">
                <label className="mode-toggle">
                  <input
                    type="checkbox"
                    checked={selectedTextOnlyMode}
                    onChange={toggleSelectedTextMode}
                  />
                  <span className="toggle-label">Selected Text Only</span>
                </label>
              </div>
            </div>
            <div className="chat-header-right">
              <button className="chat-clear-btn" onClick={clearChat} title="Clear chat">
                🗑️
              </button>
              <button className="chat-close-btn" onClick={toggleChat} title="Close chat">
                ✕
              </button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="chat-welcome">
                <p>Hello! I'm your book assistant. Ask me anything about the textbook content!</p>
                <p>You can also enable "Selected Text Only" mode to ask questions about highlighted text.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chat-message ${message.sender}-message`}
                >
                  <div className="message-content">
                    {message.isError ? (
                      <span className="error-message">{message.text}</span>
                    ) : (
                      <>
                        <p>{message.text}</p>
                        {message.citations && message.citations.length > 0 && (
                          <div className="citations">
                            <strong>Citations:</strong>
                            <ul>
                              {message.citations.map((citation, idx) => (
                                <li key={idx}>{citation.text_excerpt}</li>
                              ))}
                            </ul>
                          </div>
                        )}
                      </>
                    )}
                  </div>
                  <div className="message-timestamp">
                    {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="chat-message bot-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedTextOnlyMode && selectedText && (
            <div className="selected-text-preview">
              <small>Selected text: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</small>
            </div>
          )}

          <div className="chat-input-area">
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={selectedTextOnlyMode ? "Ask about the selected text..." : "Ask about the book content..."}
              disabled={isLoading}
              rows="1"
              className="chat-input"
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              className="chat-send-btn"
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;