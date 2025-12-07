import React from 'react';
import styles from './styles.module.css';

interface Props {
  objectives: string[];
  chapter?: string;
  collapsible?: boolean;
}

export default function LearningObjectives({ objectives, chapter, collapsible = false }: Props): React.JSX.Element {
  return (
    <div className={styles.container} role="region" aria-label="Learning Objectives">
      <h4 className={styles.title}>
        {chapter ? `Learning Objectives: ${chapter}` : 'Learning Objectives'}
      </h4>
      <ul className={styles.list}>
        {objectives.map((objective, idx) => (
          <li key={idx} className={styles.item}>
            {objective}
          </li>
        ))}
      </ul>
    </div>
  );
}
