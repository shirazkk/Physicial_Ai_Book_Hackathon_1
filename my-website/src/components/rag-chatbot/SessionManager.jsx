import React, { useState, useEffect } from 'react';

const SessionManager = ({ currentSessionId, onSessionChange, onCreateNewSession, onDeleteSession }) => {
  const [sessions, setSessions] = useState([]);
  const [showSessionList, setShowSessionList] = useState(false);

  // Mock data for now - would connect to API in real implementation
  useEffect(() => {
    // Simulate loading sessions from an API
    const mockSessions = [
      { id: 'session-1', title: 'Intro to AI Questions', lastActive: '2023-05-15', messageCount: 5 },
      { id: 'session-2', title: 'ML Algorithms Discussion', lastActive: '2023-05-16', messageCount: 12 },
      { id: 'session-3', title: 'Neural Networks Q&A', lastActive: '2023-05-17', messageCount: 8 },
    ];

    setSessions(mockSessions);
  }, []);

  const toggleSessionList = () => {
    setShowSessionList(!showSessionList);
  };

  const handleSessionSelect = (sessionId) => {
    onSessionChange(sessionId);
    setShowSessionList(false);
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
              onClick={() => {
                onCreateNewSession();
                setShowSessionList(false);
              }}
            >
              + New Session
            </button>
          </div>

          <div className="session-list">
            {sessions.length > 0 ? (
              sessions.map((session) => (
                <div
                  key={session.id}
                  className={`session-item ${session.id === currentSessionId ? 'active' : ''}`}
                  onClick={() => handleSessionSelect(session.id)}
                >
                  <div className="session-info">
                    <div className="session-title">{session.title}</div>
                    <div className="session-meta">
                      <span>{session.messageCount} messages</span>
                      <span>Last: {formatDate(session.lastActive)}</span>
                    </div>
                  </div>
                  <button
                    className="delete-session-btn"
                    onClick={(e) => {
                      e.stopPropagation();
                      onDeleteSession(session.id);
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