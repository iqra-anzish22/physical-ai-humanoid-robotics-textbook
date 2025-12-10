import React from 'react';
import clsx from 'clsx';
import styles from './LearningOutcome.module.css';

// LearningOutcome component to display learning outcomes for a chapter in a structured format
const LearningOutcome = ({ outcomes, style = 'default' }) => {
  if (!outcomes || outcomes.length === 0) {
    return null;
  }

  return (
    <div className={clsx('learning-outcome-container', styles.container, styles[style])}>
      <h3 className={styles.title}>Learning Outcomes</h3>
      <ul className={styles.outcomesList}>
        {outcomes.map((outcome, index) => (
          <li key={index} className={styles.outcomeItem}>
            <span className={styles.bullet}>•</span> {outcome}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default LearningOutcome;