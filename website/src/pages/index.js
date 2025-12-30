import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import HomepageFeatures from '../components/HomepageFeatures';
import styles from './index.module.css';

/**
 * Homepage Hero Section
 * Modern hero with dark gradient background
 */
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroContent}>
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <p className="hero__description">
          Master Physical AI and Humanoid Robotics through our comprehensive,
          simulation-first guide to embodied intelligence.
        </p>
        <div className={styles.heroButtons}>
          <Link
            className="button button--primary"
            to="/intro/physical-ai-fundamentals">
            Start Learning
          </Link>
          <Link
            className="button button--outline"
            href="https://github.com/PhysicalAI-Humanoid-Robotics/PHYSICALAI_HUMANOID_ROBOTICS"
            target="_blank"
            rel="noopener noreferrer">
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

/**
 * Section Divider
 * Visual separator between homepage sections
 */
function SectionDivider({ title, content }) {
  return (
    <section className="section-divider">
      <h2 className="section-divider__title">{title}</h2>
      <p className="section-divider__content">{content}</p>
    </section>
  );
}

/**
 * "Why This Book?" Section
 * Dark background strip with compelling messaging
 */
function WhyThisBookSection() {
  return (
    <section className="why-this-book-section">
      <div className="why-this-book-content">
        <h2>Why This Book?</h2>
        <p>
          The Physical AI landscape is rapidly evolving, with humanoid robotics emerging as one of the most exciting frontiers. This book provides a structured, simulation-first path from fundamentals to capstone-level system design.
        </p>
        <p>
          Whether you're a CS student, robotics engineer, or AI researcher, you'll learn to build autonomous humanoid systems that understand language, navigate environments, and interact naturally with humans—all without owning physical hardware.
        </p>
        <p>
          Built with Spec-Driven Development (SDD) principles, every chapter includes runnable code examples, verified citations, and progressive difficulty that builds from first principles to cutting-edge VLA (Vision-Language-Action) systems.
        </p>
      </div>
    </section>
  );
}

/**
 * Main Homepage Component
 * Complete modern course-style landing page
 */
export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive guide to Physical AI and Humanoid Robotics with simulation-first learning, ROS 2 fundamentals, NVIDIA Isaac platform, and autonomous humanoid system design.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <SectionDivider
          title="Built for Modern Developers"
          content="From ROS 2 middleware to NVIDIA Isaac GPU acceleration, from digital twins to conversational robotics, this book covers the full technology stack needed to build autonomous humanoid robots."
        />
        <WhyThisBookSection />
      </main>
    </Layout>
  );
}
