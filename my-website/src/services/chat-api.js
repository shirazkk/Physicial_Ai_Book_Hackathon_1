/**
 * API client service for communicating with the RAG chatbot backend.
 */

class ChatApiService {
  constructor() {
    // Get the API URL from environment or use default
    // Safe handling for both build time and runtime in Docusaurus
    let apiUrl = 'http://localhost:8000';

    // Check if process exists and has env (Node.js environment)
    if (typeof process !== 'undefined' && process.env) {
      apiUrl = process.env.REACT_APP_CHATBOT_API_URL || process.env.CHATBOT_API_URL || 'http://localhost:8000';
    }
    // For browser environments, we can also check for a global config
    else if (typeof window !== 'undefined' && window.APP_CONFIG) {
      apiUrl = window.APP_CONFIG.CHATBOT_API_URL || 'http://localhost:8000';
    }
    // Final fallback to localStorage for runtime configuration
    else if (typeof window !== 'undefined') {
      apiUrl = localStorage.getItem('CHATBOT_API_URL') || 'http://localhost:8000';
    }

    this.baseUrl = apiUrl;
    this.apiPrefix = '/api/v1';
  }

  /**
   * Send a question to the chatbot and get a response.
   *
   * @param {string} question - The user's question
   * @param {string|null} sessionId - Optional session ID to continue a conversation
   * @param {boolean} selectedTextOnlyMode - Whether to use selected text only mode
   * @param {string|null} selectedText - Text selected by the user (if in selected text mode)
   * @returns {Promise<Object>} Response from the chatbot
   */
  async askQuestion(question, sessionId = null, selectedTextOnlyMode = false, selectedText = null) {
    const url = `${this.baseUrl}${this.apiPrefix}/chat`;

    const requestBody = {
      question: question,
      session_id: sessionId,
      selected_text_only: selectedTextOnlyMode
    };

    // Add selected text if in selected text mode
    if (selectedTextOnlyMode && selectedText) {
      requestBody.selected_text = selectedText;
    }

    try {
      const response = await fetch(url, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error sending question to chatbot:', error);
      throw error;
    }
  }

  /**
   * Get a list of previous sessions for the user.
   *
   * @param {string} userId - The user ID
   * @returns {Promise<Array>} List of session objects
   */
  async getSessions(userId) {
    const url = `${this.baseUrl}${this.apiPrefix}/sessions?user_id=${encodeURIComponent(userId)}`;

    try {
      const response = await fetch(url, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error fetching sessions:', error);
      throw error;
    }
  }

  /**
   * Get messages from a specific session.
   *
   * @param {string} sessionId - The session ID
   * @returns {Promise<Array>} List of message objects
   */
  async getSessionMessages(sessionId) {
    const url = `${this.baseUrl}${this.apiPrefix}/sessions/${sessionId}/messages`;

    try {
      const response = await fetch(url, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error fetching session messages:', error);
      throw error;
    }
  }

  /**
   * Create a new session.
   *
   * @param {Object} sessionData - Session data including user ID
   * @returns {Promise<Object>} New session object
   */
  async createSession(sessionData) {
    const url = `${this.baseUrl}${this.apiPrefix}/sessions`;

    try {
      const response = await fetch(url, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(sessionData)
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error creating session:', error);
      throw error;
    }
  }

  /**
   * Delete a session.
   *
   * @param {string} sessionId - The session ID to delete
   * @returns {Promise<boolean>} Success status
   */
  async deleteSession(sessionId) {
    const url = `${this.baseUrl}${this.apiPrefix}/sessions/${sessionId}`;

    try {
      const response = await fetch(url, {
        method: 'DELETE',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return true;
    } catch (error) {
      console.error('Error deleting session:', error);
      throw error;
    }
  }

  /**
   * Test the API connection.
   *
   * @returns {Promise<boolean>} Connection status
   */
  async testConnection() {
    const url = `${this.baseUrl}/health`;

    try {
      const response = await fetch(url, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      return response.ok;
    } catch (error) {
      console.error('Error testing connection:', error);
      return false;
    }
  }
}

// Export a singleton instance
const chatApiService = new ChatApiService();
export default chatApiService;

// Also export the class for direct instantiation if needed
export { ChatApiService };