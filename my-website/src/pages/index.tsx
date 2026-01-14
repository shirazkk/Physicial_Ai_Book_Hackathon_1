import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import Layout from '@theme/Layout';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroInner}>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro">
              Start Reading - Introduction
            </Link>
            <Link
              className="button button--primary button--lg"
              to="/docs/module-1-foundation/chapter1-mathematical-foundations">
              Begin with Foundations
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function BookIntroSection() {
  return (
    <section className={styles.section}>
      <div className="container padding-horiz--md">
        <div className={styles.row}>
          <div className={styles.col}>
            <Heading as="h2" className={styles.sectionTitle}>
              About This Book
            </Heading>
            <p className={styles.sectionText}>
              This comprehensive textbook provides a complete guide to Physical AI and Humanoid Robotics,
              bridging the gap between theoretical foundations and practical implementation. The book follows
              a simulation-first approach, emphasizing hands-on learning through Gazebo, Unity, and the
              NVIDIA Isaac ecosystem.
            </p>
            <p className={styles.sectionText}>
              Designed for advanced undergraduate and graduate students, researchers, and engineers,
              this book covers the essential concepts needed to develop intelligent humanoid robots
              using state-of-the-art AI and robotics technologies.
            </p>
          </div>
          <div className={styles.col}>
            <div className={styles.bookCover}>
              <div className={styles.coverImage}>
                <div className={styles.robotIcon}>🤖</div>
                <div className={styles.coverTitle}>Physical AI & Humanoid Robotics</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function LearningOutcomesSection() {
  return (
    <section className={clsx(styles.section, styles.altSection)}>
      <div className="container padding-horiz--md">
        <Heading as="h2" className={styles.sectionTitle}>
          Learning Outcomes
        </Heading>
        <p className={styles.sectionSubtitle}>
          Upon completion, students will be able to:
        </p>
        <div className={styles.outcomesGrid}>
          <div className={styles.outcomeCard}>
            <div className={styles.outcomeIcon}>🔧</div>
            <h3>Configure Simulation Environments</h3>
            <p>Configure Gazebo simulation environments with accurate physics parameters for humanoid robot testing</p>
          </div>
          <div className={styles.outcomeCard}>
            <div className={styles.outcomeIcon}>🏗️</div>
            <h3>Build Robot Models</h3>
            <p>Build URDF robot models with proper kinematic chains and dynamic properties for ROS integration</p>
          </div>
          <div className={styles.outcomeCard}>
            <div className={styles.outcomeIcon}>📡</div>
            <h3>Implement Sensors</h3>
            <p>Implement sensor simulation pipelines with realistic noise models for LiDAR, cameras, and IMUs</p>
          </div>
          <div className={styles.outcomeCard}>
            <div className={styles.outcomeIcon}>🔗</div>
            <h3>Integrate Systems</h3>
            <p>Integrate Unity visualization with ROS-based control systems for human-robot interaction</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function ModulesOverviewSection() {
  return (
    <section className={styles.section}>
      <div className="container padding-horiz--md">
        <Heading as="h2" className={styles.sectionTitle}>
          Book Structure
        </Heading>
        <p className={styles.sectionSubtitle}>
          Three comprehensive modules covering essential topics
        </p>
        <div className={styles.modulesGrid}>
          <div className={styles.moduleCard}>
            <div className={styles.moduleHeader}>
              <div className={styles.moduleNumber}>01</div>
              <h3>Foundation</h3>
            </div>
            <ul className={styles.moduleTopics}>
              <li>Mathematical Foundations</li>
              <li>Kinematics and Dynamics</li>
              <li>Sensing and Perception</li>
              <li>Embodied Intelligence</li>
            </ul>
            <Link
              className="button button--outline button--primary button--block"
              to="/docs/module-1-foundation/chapter1-mathematical-foundations">
              Explore Module 1
            </Link>
          </div>

          <div className={styles.moduleCard}>
            <div className={styles.moduleHeader}>
              <div className={styles.moduleNumber}>02</div>
              <h3>Robotic Nervous System</h3>
            </div>
            <ul className={styles.moduleTopics}>
              <li>ROS 2 Architecture</li>
              <li>AI-Agent Bridges</li>
              <li>URDF Humanoid Description</li>
              <li>Nervous System Patterns</li>
            </ul>
            <Link
              className="button button--outline button--primary button--block"
              to="/docs/module-2-robotic-nervous-system/chapter-1/ros2-architecture">
              Explore Module 2
            </Link>
          </div>

          <div className={styles.moduleCard}>
            <div className={styles.moduleHeader}>
              <div className={styles.moduleNumber}>03</div>
              <h3>Digital Twin</h3>
            </div>
            <ul className={styles.moduleTopics}>
              <li>Gazebo Simulation</li>
              <li>Unity Visualization</li>
              <li>Isaac Ecosystem</li>
              <li>Integration Patterns</li>
            </ul>
            <Link
              className="button button--outline button--primary button--block"
              to="/docs/module-3-digital-twin/chapter-1-gazebo-basics/content">
              Explore Module 3
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home | ${siteConfig.title}`}
      description="Comprehensive guide to Physical AI & Humanoid Robotics - From foundational concepts to advanced robotic nervous systems">
      <HomepageHeader />
      <main>
        <BookIntroSection />
        <ModulesOverviewSection />
        <LearningOutcomesSection />
    
      </main>
    </Layout>
  );
}