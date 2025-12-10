import React from 'react';
import clsx from 'clsx';
import styles from './Exercise.module.css';

// Exercise component to display interactive exercises with instructions and expected outcomes
const Exercise = ({ title, difficulty, type, instructions, expectedOutcome }) => {
  const getDifficultyClass = (difficulty) => {
    switch (difficulty?.toLowerCase()) {
      case 'beginner':
        return styles.beginner;
      case 'intermediate':
        return styles.intermediate;
      case 'advanced':
        return styles.advanced;
      default:
        return styles.default;
    }
  };

  const getTypeLabel = (type) => {
    switch (type?.toLowerCase()) {
      case 'theoretical':
        return 'Theoretical';
      case 'practical':
        return 'Practical';
      case 'simulation':
        return 'Simulation';
      case 'hardware':
        return 'Hardware';
      default:
        return type || 'Exercise';
    }
  };

  return (
    <div className={clsx('exercise-container', styles.container)}>
      <div className={styles.header}>
        <h3 className={styles.title}>{title || 'Exercise'}</h3>
        <div className={styles.meta}>
          <span className={clsx(styles.difficulty, getDifficultyClass(difficulty))}>
            {difficulty || 'Unknown Level'}
          </span>
          <span className={styles.type}>{getTypeLabel(type)}</span>
        </div>
      </div>

      <div className={styles.content}>
        <h4 className={styles.sectionTitle}>Instructions:</h4>
        <div className={styles.instructions}>
          {instructions ? (
            <div dangerouslySetInnerHTML={{ __html: instructions }} />
          ) : (
            <p>No instructions provided.</p>
          )}
        </div>

        {expectedOutcome && (
          <>
            <h4 className={styles.sectionTitle}>Expected Outcome:</h4>
            <div className={styles.expectedOutcome}>
              {expectedOutcome}
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export default Exercise;