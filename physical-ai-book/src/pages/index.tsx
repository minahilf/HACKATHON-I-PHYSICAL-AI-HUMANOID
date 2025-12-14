import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './index.module.css';

// Component to render the Spline scene only in the browser
const SplineScene = () => (
  <BrowserOnly fallback={<div>Loading 3D experience...</div>}>
    {() => {
      const Spline = require('@splinetool/react-spline').default;
      return <Spline scene="https://prod.spline.design/6Wq1Q7YGyM-iab9i/scene.splinecode" />;
    }}
  </BrowserOnly>
);

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={clsx('row', styles.heroContent)}>
          <div className={clsx('col col--6', styles.heroText)}>
            <h1 className={clsx('hero__title', styles.neonGradient)}>
              {siteConfig.title}
            </h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Explore the Textbook - 5min ⏱️
              </Link>
            </div>
          </div>
          <div className={clsx('col col--6', styles.hero3D)}>
            <SplineScene />
          </div>
        </div>
      </div>
    </header>
  );
}

const ModuleCard = ({title, description, link}) => {
  return (
    <Link to={link} className={clsx('card', styles.moduleCard)}>
      <h3>{title}</h3>
      <p>{description}</p>
    </Link>
  );
};

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();

  const modules = [
    {
      title: 'Module 1: ROS 2 Nervous System',
      description: 'Understand the core communication paradigms of ROS 2.',
      link: '/docs/01-module-1-ros2/01-nodes-intro',
    },
    {
      title: 'Module 2: The Digital Twin',
      description: 'Dive into robot simulation with Gazebo and Unity.',
      link: '/docs/02-module-2-digital-twin/01-physics-in-gazebo',
    },
    {
      title: 'Module 3: The AI-Robot Brain',
      description: 'Explore NVIDIA Isaac for AI-powered robotics.',
      link: '/docs/03-module-3-isaac/01-isaac-sim-intro',
    },
    {
      title: 'Module 4: Vision-Language-Action (VLA)',
      description: 'Integrate advanced AI for human-robot collaboration.',
      link: '/docs/04-module-4-vla/01-voice-to-action',
    },
  ];

  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="The Physical AI Textbook - Learn by doing.">
      <HomepageHeader />
      <main className={styles.curriculumSection}>
        <div className="container">
          <h2 className={styles.sectionTitle}>Our Curriculum</h2>
          <div className={clsx('row', styles.moduleGrid)}>
            {modules.map((props, idx) => (
              <ModuleCard key={idx} {...props} />
            ))}
          </div>
        </div>
      </main>
    </Layout>
  );
}
