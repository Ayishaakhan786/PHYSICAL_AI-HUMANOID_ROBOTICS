import React from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

/**
 * HomepageFeatures Component
 * Displays a grid of feature cards with modern course-style design
 */

function FeatureCard({ icon, title, description }) {
  return (
    <div className="feature-card">
      <div className="feature-card__icon">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path strokeLinecap="round" strokeLinejoin="round" d="M12 2l2-5 2l8 5a5 5h5a6 5l8 3a6 5l8 4a6 5l8 5l8 6a5" />
          <path strokeLinecap="round" strokeLinejoin="round" d="M13 5l7-10 10l7-14 14l8 3l8 4a6 4l8 5l8 6a5" />
          <path strokeLinecap="round" strokeLinejoin="round" d="M2 12l9 5l9 14l8 7l8 10l8 2l7 2a3" />
        </svg>
      </div>
      <h3 className="feature-card__title">{title}</h3>
      <p className="feature-card__description">{description}</p>
    </div>
  );
}

export default function HomepageFeatures() {
  const features = [
    {
      icon: 'brain',
      title: 'Physical AI Fundamentals',
      description: 'Master embodied intelligence, simulation-first workflows, and the foundational concepts that power modern robotics.',
    },
    {
      icon: 'network',
      title: 'ROS 2 Architecture',
      description: 'Learn the industry-standard middleware that connects perception, planning, and actuation in robotic systems through nodes, topics, and services.',
    },
    {
      icon: 'cube',
      title: 'Digital Twin Simulation',
      description: 'Explore physics engines and sensor modeling in virtual environments. Understand Gazebo, Unity, and the digital twin approach to safe, efficient development.',
    },
    {
      icon: 'chip',
      title: 'NVIDIA Isaac Platform',
      description: 'Discover GPU-accelerated robotics workflows, VSLAM navigation, and perception pipelines that enable faster-than-real-time training.',
    },
    {
      icon: 'message-square',
      title: 'VLA & Conversational Robotics',
      description: 'Integrate vision, language, and action models with robotic control. Build robots that understand natural language and interact intuitively.',
    },
    {
      icon: 'robot',
      title: 'Humanoid Robotics',
      description: 'Study kinematics, locomotion, and human-robot interaction design. Understand how humanoid robots move, balance, and collaborate with humans.',
    },
    {
      icon: 'layers',
      title: 'Hardware & Labs',
      description: 'Explore compute requirements for workstations and edge devices. Compare on-premise labs with cloud solutions for your development needs.',
    },
    {
      icon: 'flag',
      title: 'Sim-to-Real Challenges',
      description: 'Understand domain randomization, the reality gap, and transfer learning techniques for deploying simulation-trained algorithms to physical robots.',
    },
  ];

  return (
    <section className="feature-cards-section">
      <h2 className="section-title">What You Will Learn</h2>
      <div className="feature-cards-grid">
        {features.map((feature, index) => (
          <FeatureCard
            key={index}
            icon={feature.icon}
            title={feature.title}
            description={feature.description}
          />
        ))}
      </div>
    </section>
  );
}
