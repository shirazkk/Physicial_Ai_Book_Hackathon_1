import React from 'react';
import clsx from 'clsx';
import styles from './IsaacIntegrationBox.module.css';

type IsaacIntegrationBoxProps = {
  children: React.ReactNode;
  title?: string;
  type?: 'note' | 'tip' | 'warning' | 'caution';
};

const IsaacIntegrationBox: React.FC<IsaacIntegrationBoxProps> = ({
  children,
  title = 'Isaac Ecosystem Integration',
  type = 'note'
}) => {
  return (
    <div className={clsx(
      styles.integrationBox,
      styles[`integrationBox${type.charAt(0).toUpperCase() + type.slice(1)}`]
    )}>
      <div className={styles.integrationHeader}>
        <h4 className={styles.integrationTitle}>{title}</h4>
      </div>
      <div className={styles.integrationContent}>
        {children}
      </div>
    </div>
  );
};

export default IsaacIntegrationBox;