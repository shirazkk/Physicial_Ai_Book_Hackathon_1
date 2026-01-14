import React, { useState, useEffect } from 'react';
import chatApiService from '../../services/chat-api';

const SessionManager = ({ currentSessionId, onSessionChange, onCreateNewSession, onDeleteSession, userId = 'default-user' }) => {
  const [sessions, setSessions] = useState([]);
  const [showSessionList, setShowSessionList] = useState(false);
  const [loading, setLoading] = useState(false);

  // Load sessions from API
  useEffect(() => {
    loadSessions();
  }, [userId]);

  const loadSessions = async () => {
    setLoading(true);
    try {
      const sessionList = await chatApiService.getSessions(userId);
      setSessions(sessionList.sessions || []);
    } catch (error) {
      console.error('Error loading sessions:', error);
      // Set empty array on error, could show error message to user
      setSessions([]);
    } finally {
      setLoading(false);
    }
  };

  const toggleSessionList = () => {
    setShowSessionList(!showSessionList);
  };

  const handleSessionSelect = (sessionId) => {
    onSessionChange(sessionId);
    setShowSessionList(false);
  };

  const handleCreateNewSession = async () => {
    try {
      const newSession = await chatApiService.createSession({ user_id: userId });
      // Refresh the session list
      await loadSessions();
      // Switch to the new session
      onSessionChange(newSession.session_id);
    } catch (error) {
      console.error('Error creating new session:', error);
    }
    setShowSessionList(false);
  };

  const handleDeleteSession = async (sessionId) => {
    try {
      await chatApiService.deleteSession(sessionId);
      // Refresh the session list
      await loadSessions();
      // If we deleted the current session, switch to a different one or clear current session
      if (currentSessionId === sessionId) {
        onSessionChange(null);
      }
    } catch (error) {
      console.error('Error deleting session:', error);
    }
  };

  const formatDate = (dateString) => {
    const date = new Date(dateString);
    return date.toLocaleDateString();
  };

  return (
    <div className="session-manager">
      <button className="session-dropdown-btn" onClick={toggleSessionList}>
        {currentSessionId ? `Session: ${currentSessionId}` : 'Select Session'} ▼
      </button>

      {showSessionList && (
        <div className="session-list-dropdown">
          <div className="session-actions">
            <button
              className="new-session-btn"
              onClick={handleCreateNewSession}
              disabled={loading}
            >
              {loading ? 'Creating...' : '+ New Session'}
            </button>
          </div>

          <div className="session-list">
            {loading ? (
              <div className="loading-sessions">Loading sessions...</div>
            ) : sessions.length > 0 ? (
              sessions.map((session) => (
                <div
                  key={session.session_id}
                  className={`session-item ${session.session_id === currentSessionId ? 'active' : ''}`}
                  onClick={() => handleSessionSelect(session.session_id)}
                >
                  <div className="session-info">
                    <div className="session-title">{session.title}</div>
                    <div className="session-meta">
                      <span>{session.message_count} messages</span>
                      <span>Last: {formatDate(session.last_active)}</span>
                    </div>
                  </div>
                  <button
                    className="delete-session-btn"
                    onClick={(e) => {
                      e.stopPropagation();
                      handleDeleteSession(session.session_id);
                    }}
                    title="Delete session"
                  >
                    ×
                  </button>
                </div>
              ))
            ) : (
              <div className="no-sessions">No sessions available</div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default SessionManager;