import React, { useState, useEffect } from 'react';

const TextSelector = ({ onTextSelected }) => {
  const [selectedText, setSelectedText] = useState('');
  const [showNotification, setShowNotification] = useState(false);

  useEffect(() => {
    const handleSelection = () => {
      const selectedTextContent = window.getSelection().toString().trim();
      setSelectedText(selectedTextContent);

      if (selectedTextContent) {
        // Show notification briefly
        setShowNotification(true);
        setTimeout(() => setShowNotification(false), 2000);
      }

      // Notify parent component of text selection
      if (onTextSelected) {
        onTextSelected(selectedTextContent);
      }
    };

    document.addEventListener('mouseup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, [onTextSelected]);

  return (
    <div className="text-selector-indicator">
      {selectedText && (
        <div className={`selection-notification ${showNotification ? 'visible' : 'hidden'}`}>
          <span>Selected: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"</span>
        </div>
      )}
    </div>
  );
};

export default TextSelector;