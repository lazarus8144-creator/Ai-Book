import React from 'react';
import styles from './styles.module.css';

interface HardwareItem {
  name: string;
  category: 'compute' | 'sensor' | 'actuator' | 'network' | 'software';
  minimumSpecs: string;
  recommendedSpecs: string;
  modules: number[];
  estimatedCost?: string;
  required: boolean;
  purchaseLink?: string;
}

interface Props {
  items: HardwareItem[];
  showCost?: boolean;
  filterByModule?: number;
}

export default function HardwareRequirements({ items, showCost = true, filterByModule }: Props): React.JSX.Element {
  const filteredItems = filterByModule
    ? items.filter(item => item.modules.includes(filterByModule))
    : items;

  const categoryLabels = {
    compute: '💻 Computing',
    sensor: '👁️ Sensors',
    actuator: '🦾 Actuators',
    network: '🌐 Network',
    software: '💿 Software'
  };

  return (
    <div className={styles.container} role="region" aria-label="Hardware Requirements">
      <h4 className={styles.title}>Hardware Requirements</h4>
      <div className={styles.table} role="table" aria-label="Hardware specifications">
        <div className={styles.tableHeader} role="rowgroup">
          <div className={styles.headerCell} role="columnheader">Item</div>
          <div className={styles.headerCell} role="columnheader">Minimum Specs</div>
          <div className={styles.headerCell} role="columnheader">Recommended Specs</div>
          {showCost && <div className={styles.headerCell} role="columnheader">Est. Cost</div>}
          <div className={styles.headerCell} role="columnheader">Status</div>
        </div>
        {filteredItems.map((item, idx) => (
          <div key={idx} className={styles.tableRow} role="row">
            <div className={styles.cell} role="cell" data-label="Item">
              <span className={styles.categoryBadge}>{categoryLabels[item.category]}</span>
              <span className={styles.itemName}>{item.name}</span>
            </div>
            <div className={styles.cell} role="cell" data-label="Minimum Specs">
              {item.minimumSpecs}
            </div>
            <div className={styles.cell} role="cell" data-label="Recommended Specs">
              {item.recommendedSpecs}
            </div>
            {showCost && (
              <div className={styles.cell} role="cell" data-label="Est. Cost">
                {item.estimatedCost || 'N/A'}
              </div>
            )}
            <div className={styles.cell} role="cell" data-label="Status">
              <span className={item.required ? styles.required : styles.optional}>
                {item.required ? 'Required' : 'Optional'}
              </span>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
