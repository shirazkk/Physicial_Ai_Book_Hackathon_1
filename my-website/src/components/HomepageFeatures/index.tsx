import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg?: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  icon?: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Foundational Concepts',
    icon: '🧮',
    description: (
      <>
        Master the mathematical foundations, kinematics, dynamics, sensing, perception, and embodied intelligence
        that form the core of Physical AI and humanoid robotics.
      </>
    ),
  },
  {
    title: 'Robotic Nervous System',
    icon: '⚡',
    description: (
      <>
        Learn about ROS 2 architecture, AI-agent bridges, URDF humanoid descriptions, and robotic nervous system patterns
        for creating intelligent robots.
      </>
    ),
  },
  {
    title: 'Digital Twin & Simulation',
    icon: '🎮',
    description: (
      <>
        Discover how to leverage Gazebo, Unity, and NVIDIA Isaac ecosystems for simulation-first development
        and real-world deployment of humanoid robots.
      </>
    ),
  },
];

function Feature({title, description, icon}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <div className={styles.featureIcon}>{icon}</div>
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
