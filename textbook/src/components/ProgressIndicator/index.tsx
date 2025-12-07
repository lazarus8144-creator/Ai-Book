import React from 'react';
import styles from './styles.module.css';

interface Props {
  currentModule: number;
  currentChapter: number;
  totalChapters: number;
}

export default function ProgressIndicator({ currentModule, currentChapter, totalChapters }: Props): React.JSX.Element {
  const progressPercentage = (currentChapter / totalChapters) * 100;

  const moduleNames = {
    1: 'ROS 2',
    2: 'Digital Twin',
    3: 'NVIDIA Isaac',
    4: 'VLA'
  };

  return (
    <div className={styles.container} role="region" aria-label="Module Progress">
      <div className={styles.header}>
        <span className={styles.moduleLabel}>Module {currentModule}: {moduleNames[currentModule]}</span>
        <span className={styles.chapterLabel}>Chapter {currentChapter} of {totalChapters}</span>
      </div>
      <div className={styles.progressBar} role="progressbar" aria-valuenow={progressPercentage} aria-valuemin={0} aria-valuemax={100}>
        <div
          className={styles.progressFill}
          style={{ width: `${progressPercentage}%` }}
        />
      </div>
      <div className={styles.percentage}>{Math.round(progressPercentage)}% Complete</div>
    </div>
  );
}
