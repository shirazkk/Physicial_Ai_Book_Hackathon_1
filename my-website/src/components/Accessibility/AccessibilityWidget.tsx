import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './AccessibilityWidget.module.css';

const AccessibilityWidget: React.FC = () => {
  const [highContrast, setHighContrast] = useState(false);
  const [fontSize, setFontSize] = useState(1);

  useEffect(() => {
    if (highContrast) {
      document.body.classList.add(styles.highContrastMode);
    } else {
      document.body.classList.remove(styles.highContrastMode);
    }

    document.documentElement.style.fontSize = `${fontSize * 16}px`;
  }, [highContrast, fontSize]);

  const increaseFontSize = () => {
    if (fontSize < 1.5) {
      setFontSize(fontSize + 0.1);
    }
  };

  const decreaseFontSize = () => {
    if (fontSize > 0.8) {
      setFontSize(fontSize - 0.1);
    }
  };

  const resetSettings = () => {
    setHighContrast(false);
    setFontSize(1);
  };

  return (
    <div className={clsx(styles.accessibilityWidget, 'padding--sm', 'margin-bottom--md')}>
      <h3 className={styles.widgetTitle}>Accessibility Tools</h3>
      <div className={styles.controls}>
        <button
          onClick={() => setHighContrast(!highContrast)}
          className={clsx('button', 'button--secondary', styles.controlButton)}
          aria-pressed={highContrast}
        >
          {highContrast ? 'Disable' : 'Enable'} High Contrast
        </button>

        <div className={styles.fontSizeControls}>
          <button
            onClick={decreaseFontSize}
            className={clsx('button', 'button--secondary', styles.controlButton)}
            disabled={fontSize <= 0.8}
            aria-label="Decrease font size"
          >
            A-
          </button>

          <span className={styles.fontSizeLabel}>Font Size: {Math.round(fontSize * 100)}%</span>

          <button
            onClick={increaseFontSize}
            className={clsx('button', 'button--secondary', styles.controlButton)}
            disabled={fontSize >= 1.5}
            aria-label="Increase font size"
          >
            A+
          </button>
        </div>

        <button
          onClick={resetSettings}
          className={clsx('button', 'button--outline', styles.resetButton)}
        >
          Reset Settings
        </button>
      </div>

      <div className={styles.keyboardInfo}>
        <p>All content is accessible via keyboard navigation.</p>
        <p>Press <kbd>Tab</kbd> to navigate, <kbd>Enter</kbd> or <kbd>Space</kbd> to activate controls.</p>
      </div>
    </div>
  );
};

export default AccessibilityWidget;