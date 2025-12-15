import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

// Hero Section (Upar wala hissa)
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            The Premier <br />
            <span className={styles.gradientText}>Physical AI & Humanoid</span> <br />
            Hackathon Book
          </h1>
          <p className={styles.heroSubtitle}>
            Master the future of robotics. Dive into ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action models to build the next generation of intelligent humanoid robots.
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/category/module-1-ros-2-nervous-system">
              Explore the Textbook
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

// Module Card Component (Neeche wale cards)
const ModuleCard = ({title, description, link}) => {
  return (
    <Link to={link} className={clsx('card', styles.moduleCard)}>
      <h3>{title}</h3>
      <p>{description}</p>
    </Link>
  );
};

// Main Page Component
export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  const modules = [
    {
      title: 'Module 1: ROS 2 Nervous System',
      description: 'Understand the core communication paradigms of ROS 2.',
      link: '/docs/category/module-1-ros-2-nervous-system',
    },
    {
      title: 'Module 2: The Digital Twin',
      description: 'Dive into robot simulation with Gazebo and Unity.',
      link: '/docs/module-2-digital-twin/physics-in-gazebo',
    },
    {
      title: 'Module 3: The AI-Robot Brain',
      description: 'Explore NVIDIA Isaac for AI-powered robotics.',
      link: '/docs/module-3-isaac/isaac-sim-intro',
    },
    {
      title: 'Module 4: Vision-Language-Action (VLA)',
      description: 'Integrate advanced AI for human-robot collaboration.',
      link: '/docs/module-4-vla/voice-to-action',
    },
  ];

  return (
    <Layout
      title={siteConfig.title}
      description="The definitive guide to Physical AI and Humanoid Robotics.">
      <HomepageHeader />
      <main>
        <div className={styles.curriculumSection}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Our Curriculum</h2>
            <div className={styles.moduleGrid}>
              {modules.map((props, idx) => (
                <ModuleCard key={idx} {...props} />
              ))}
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}