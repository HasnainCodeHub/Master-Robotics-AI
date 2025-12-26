import { type ReactNode, useEffect, useRef, useState } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import clsx from 'clsx';
import styles from './index.module.css';
import DeveloperSection from '../components/DeveloperSection';

// Feature data
const features = [
  {
    title: 'Embodied AI Fundamentals',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
        <circle cx="12" cy="12" r="3" />
        <path d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42" />
      </svg>
    ),
    description: 'Master the core concepts of physical AI, from sensor integration to actuator control and real-world constraints.',
    gradient: 'linear-gradient(135deg, #0d9488, #06b6d4)',
  },
  {
    title: 'ROS 2 Mastery',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
        <rect x="2" y="3" width="20" height="14" rx="2" />
        <path d="M8 21h8M12 17v4" />
      </svg>
    ),
    description: 'Build production-ready robotic systems with ROS 2 topics, services, actions, and advanced integration patterns.',
    gradient: 'linear-gradient(135deg, #6366f1, #8b5cf6)',
  },
  {
    title: 'Simulation & Digital Twin',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
        <path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z" />
        <polyline points="3.27 6.96 12 12.01 20.73 6.96" />
        <line x1="12" y1="22.08" x2="12" y2="12" />
      </svg>
    ),
    description: 'Bridge the reality gap with Gazebo, Unity, and NVIDIA Isaac Sim for safe development and testing.',
    gradient: 'linear-gradient(135deg, #f59e0b, #f97316)',
  },
  {
    title: 'Vision-Language-Action',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
        <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
        <circle cx="12" cy="12" r="3" />
      </svg>
    ),
    description: 'Integrate cutting-edge AI with speech recognition, LLM planning, and vision-language models.',
    gradient: 'linear-gradient(135deg, #ec4899, #f43f5e)',
  },
  {
    title: 'AI-Powered Tutor',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        <path d="M8 10h.01M12 10h.01M16 10h.01" />
      </svg>
    ),
    description: 'Get instant help from our AI chatbot that answers questions strictly from textbook content.',
    gradient: 'linear-gradient(135deg, #10b981, #14b8a6)',
  },
  {
    title: 'Personalized Learning',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
        <path d="M16 21v-2a4 4 0 0 0-4-4H6a4 4 0 0 0-4 4v2" />
        <circle cx="9" cy="7" r="4" />
        <path d="M22 21v-2a4 4 0 0 0-3-3.87M16 3.13a4 4 0 0 1 0 7.75" />
      </svg>
    ),
    description: 'Content adapts to your experience level with beginner hints and advanced deep-dives.',
    gradient: 'linear-gradient(135deg, #3b82f6, #6366f1)',
  },
];

// Module data
const modules = [
  {
    number: '01',
    title: 'Physical AI Foundations',
    chapters: 4,
    description: 'Embodied intelligence, sensors, actuators, and physical constraints',
    color: '#0d9488',
    icon: '🤖',
  },
  {
    number: '02',
    title: 'ROS 2 Fundamentals',
    chapters: 6,
    description: 'Topics, services, actions, URDF, and integration patterns',
    color: '#6366f1',
    icon: '⚙️',
  },
  {
    number: '03',
    title: 'Simulation & Digital Twin',
    chapters: 6,
    description: 'Gazebo, Unity, reality gap, and domain randomization',
    color: '#f59e0b',
    icon: '🎮',
  },
  {
    number: '04',
    title: 'NVIDIA Isaac Ecosystem',
    chapters: 6,
    description: 'Isaac Sim, Visual SLAM, and sim-to-real transfer',
    color: '#76b900',
    icon: '🚀',
  },
  {
    number: '05',
    title: 'Vision-Language-Action',
    chapters: 6,
    description: 'Speech recognition, LLM planning, and grounding',
    color: '#ec4899',
    icon: '👁️',
  },
];

// Tech stack with SVG logos
const technologies = [
  { name: 'ROS 2', color: '#22314E' },
  { name: 'Python', color: '#3776AB' },
  { name: 'NVIDIA', color: '#76B900' },
  { name: 'Gazebo', color: '#F58113' },
  { name: 'Unity', color: '#000000' },
  { name: 'Claude AI', color: '#D97757' },
];

// Stats with icons
const stats = [
  { value: '6', label: 'Modules', icon: '📚' },
  { value: '34+', label: 'Chapters', icon: '📖' },
  { value: '100+', label: 'Examples', icon: '💻' },
  { value: '1', label: 'Capstone', icon: '🏆' },
];

// Hook for scroll animations
function useScrollAnimation(threshold = 0.1) {
  const ref = useRef<HTMLDivElement>(null);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
        }
      },
      { threshold }
    );

    if (ref.current) {
      observer.observe(ref.current);
    }

    return () => observer.disconnect();
  }, [threshold]);

  return { ref, isVisible };
}

// Animated counter hook
function useCounter(end: number, duration = 2000, start = 0) {
  const [count, setCount] = useState(start);
  const [isAnimating, setIsAnimating] = useState(false);

  useEffect(() => {
    if (!isAnimating) return;

    let startTime: number | null = null;
    const animate = (currentTime: number) => {
      if (!startTime) startTime = currentTime;
      const progress = Math.min((currentTime - startTime) / duration, 1);
      setCount(Math.floor(progress * (end - start) + start));
      if (progress < 1) {
        requestAnimationFrame(animate);
      }
    };
    requestAnimationFrame(animate);
  }, [isAnimating, end, start, duration]);

  return { count, startAnimation: () => setIsAnimating(true) };
}

function HeroSection(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      setMousePosition({
        x: (e.clientX / window.innerWidth - 0.5) * 20,
        y: (e.clientY / window.innerHeight - 0.5) * 20,
      });
    };
    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, []);

  return (
    <header className={styles.hero}>
      {/* Animated background */}
      <div className={styles.heroBackground}>
        <div className={styles.heroGradient} />
        <div className={styles.heroMesh} />
        <div className={styles.heroGrid} />
        <div className={styles.heroOrbs}>
          <div
            className={styles.heroOrb1}
            style={{ transform: `translate(${mousePosition.x}px, ${mousePosition.y}px)` }}
          />
          <div
            className={styles.heroOrb2}
            style={{ transform: `translate(${-mousePosition.x}px, ${-mousePosition.y}px)` }}
          />
          <div
            className={styles.heroOrb3}
            style={{ transform: `translate(${mousePosition.x * 0.5}px, ${mousePosition.y * 0.5}px)` }}
          />
        </div>
        {/* Floating particles */}
        <div className={styles.heroParticles}>
          {[...Array(20)].map((_, i) => (
            <div key={i} className={styles.particle} style={{ '--delay': `${i * 0.5}s` } as React.CSSProperties} />
          ))}
        </div>
      </div>

      <div className="container">
        <div className={styles.heroContent}>
          {/* Badge with glow */}
          <div className={styles.heroBadge}>
            <span className={styles.heroBadgeDot} />
            <span className={styles.heroBadgeText}>AI-Native Interactive Textbook</span>
            <span className={styles.heroBadgeNew}>NEW</span>
          </div>

          {/* Main title with typing effect */}
          <Heading as="h1" className={styles.heroTitle}>
            <span className={styles.heroTitleLine}>Master</span>
            <span className={styles.heroTitleAccent}> Physical AI</span>
            <br />
            <span className={styles.heroTitleLine}>&amp; Humanoid Robotics</span>
          </Heading>

          {/* Subtitle with gradient border */}
          <p className={styles.heroSubtitle}>
            Build intelligent robotic systems that operate in the physical world.
            <br className={styles.hideOnMobile} />
            From foundational concepts to voice-commanded humanoid robots.
          </p>

          {/* CTA Buttons with effects */}
          <div className={styles.heroActions}>
            <Link
              className={clsx('button button--lg', styles.heroPrimaryBtn)}
              to="/textbook/intro">
              <span className={styles.btnShine} />
              <span className={styles.btnContent}>
                Start Learning
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M5 12h14M12 5l7 7-7 7" />
                </svg>
              </span>
            </Link>
            <Link
              className={clsx('button button--lg', styles.heroSecondaryBtn)}
              to="/textbook/module-1">
              <span>Browse Curriculum</span>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M7 17L17 7M17 7H7M17 7v10" />
              </svg>
            </Link>
          </div>

          {/* Stats with icons */}
          <div className={styles.heroStats}>
            {stats.map((stat, idx) => (
              <div key={idx} className={styles.heroStat}>
                <span className={styles.heroStatIcon}>{stat.icon}</span>
                <span className={styles.heroStatValue}>{stat.value}</span>
                <span className={styles.heroStatLabel}>{stat.label}</span>
              </div>
            ))}
          </div>

          {/* Scroll indicator */}
          <div className={styles.scrollIndicator}>
            <div className={styles.scrollMouse}>
              <div className={styles.scrollWheel} />
            </div>
            <span>Scroll to explore</span>
          </div>
        </div>
      </div>

      {/* Wave separator */}
      <div className={styles.heroWave}>
        <svg viewBox="0 0 1440 120" preserveAspectRatio="none">
          <path d="M0,64 C480,150 960,-20 1440,64 L1440,120 L0,120 Z" fill="currentColor" />
        </svg>
      </div>
    </header>
  );
}

function TechStackSection(): ReactNode {
  const { ref, isVisible } = useScrollAnimation();

  return (
    <section className={styles.techStack} ref={ref}>
      <div className="container">
        <div className={styles.techStackInner}>
          <span className={styles.techStackLabel}>Powered by industry-leading technologies</span>
          <div className={clsx(styles.techLogos, isVisible && styles.visible)}>
            {technologies.map((tech, idx) => (
              <div
                key={tech.name}
                className={styles.techLogo}
                style={{ '--delay': `${idx * 100}ms` } as React.CSSProperties}
              >
                <span className={styles.techDot} style={{ background: tech.color }} />
                <span className={styles.techName}>{tech.name}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

function FeaturesSection(): ReactNode {
  const { ref, isVisible } = useScrollAnimation();

  return (
    <section className={styles.features} ref={ref}>
      <div className="container">
        <div className={clsx(styles.sectionHeader, isVisible && styles.visible)}>
          <span className={styles.sectionLabel}>Why This Course</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Everything You Need to Build
            <br />
            <span className={styles.gradientText}>Intelligent Robots</span>
          </Heading>
          <p className={styles.sectionSubtitle}>
            A comprehensive curriculum designed for hands-on learning with modern robotics tools and AI integration.
          </p>
        </div>

        <div className={clsx(styles.featuresGrid, isVisible && styles.visible)}>
          {features.map((feature, idx) => (
            <div
              key={idx}
              className={styles.featureCard}
              style={{ '--delay': `${idx * 100}ms`, '--gradient': feature.gradient } as React.CSSProperties}
            >
              <div className={styles.featureGlow} />
              <div className={styles.featureIcon}>
                {feature.icon}
              </div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
              <div className={styles.featureArrow}>
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M7 17L17 7M17 7H7M17 7v10" />
                </svg>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ModulesSection(): ReactNode {
  const { ref, isVisible } = useScrollAnimation();

  return (
    <section className={styles.modules} ref={ref}>
      <div className="container">
        <div className={clsx(styles.sectionHeader, isVisible && styles.visible)}>
          <span className={styles.sectionLabel}>Curriculum</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Structured Learning Path
          </Heading>
          <p className={styles.sectionSubtitle}>
            Progress from fundamentals to advanced topics with our carefully designed modules.
          </p>
        </div>

        {/* Timeline layout */}
        <div className={clsx(styles.modulesTimeline, isVisible && styles.visible)}>
          <div className={styles.timelineLine} />
          {modules.map((module, idx) => (
            <Link
              key={idx}
              to={`/textbook/module-${idx + 1}`}
              className={styles.moduleCard}
              style={{ '--module-color': module.color, '--delay': `${idx * 150}ms` } as React.CSSProperties}
            >
              <div className={styles.moduleConnector}>
                <div className={styles.moduleNode}>
                  <span>{module.icon}</span>
                </div>
              </div>
              <div className={styles.moduleBody}>
                <div className={styles.moduleHeader}>
                  <span className={styles.moduleNumber}>Module {module.number}</span>
                  <span className={styles.moduleChapters}>
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                      <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20" />
                      <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z" />
                    </svg>
                    {module.chapters} Chapters
                  </span>
                </div>
                <h3 className={styles.moduleTitle}>{module.title}</h3>
                <p className={styles.moduleDescription}>{module.description}</p>
                <div className={styles.moduleProgress}>
                  <div className={styles.moduleProgressBar} style={{ '--progress': '0%' } as React.CSSProperties} />
                </div>
              </div>
              <div className={styles.moduleArrow}>
                <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M5 12h14M12 5l7 7-7 7" />
                </svg>
              </div>
            </Link>
          ))}
        </div>

        {/* Capstone Card */}
        <Link to="/textbook/capstone" className={clsx(styles.capstoneCard, isVisible && styles.visible)}>
          <div className={styles.capstoneGlow} />
          <div className={styles.capstoneIcon}>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
              <polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2" />
            </svg>
          </div>
          <div className={styles.capstoneContent}>
            <span className={styles.capstoneLabel}>Capstone Project</span>
            <h3 className={styles.capstoneTitle}>Build a Voice-Commanded Service Robot</h3>
            <p className={styles.capstoneDescription}>
              Apply everything you've learned to build a complete robotic system with voice control, navigation, and manipulation.
            </p>
            <ul className={styles.capstoneSkills}>
              <li>Voice Recognition</li>
              <li>Path Planning</li>
              <li>Object Detection</li>
              <li>Manipulation</li>
            </ul>
          </div>
          <div className={styles.capstoneArrow}>
            <span>Start Capstone</span>
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M5 12h14M12 5l7 7-7 7" />
            </svg>
          </div>
        </Link>
      </div>
    </section>
  );
}

function AIFeaturesSection(): ReactNode {
  const { ref, isVisible } = useScrollAnimation();

  return (
    <section className={styles.aiFeatures} ref={ref}>
      <div className="container">
        <div className={clsx(styles.sectionHeader, isVisible && styles.visible)}>
          <span className={styles.sectionLabel}>AI-Powered Learning</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Learn Smarter with
            <br />
            <span className={styles.gradientText}>Intelligent Assistance</span>
          </Heading>
        </div>

        <div className={clsx(styles.aiFeaturesGrid, isVisible && styles.visible)}>
          {/* AI Tutor Card - Featured */}
          <div className={clsx(styles.aiCard, styles.aiCardFeatured)}>
            <div className={styles.aiCardGlow} />
            <div className={styles.aiCardIcon}>
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
              </svg>
            </div>
            <div className={styles.aiCardBadge}>RAG-Powered</div>
            <h3 className={styles.aiCardTitle}>AI-Powered Tutoring</h3>
            <p className={styles.aiCardDescription}>
              Ask questions and get instant answers grounded in the textbook content.
              Our RAG-based chatbot ensures accurate, curriculum-aligned responses.
            </p>
            <ul className={styles.aiCardFeatures}>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                Answers from textbook only
              </li>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                Source citations included
              </li>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                Context-aware responses
              </li>
            </ul>
            <div className={styles.aiCardDemo}>
              <div className={styles.chatPreview}>
                <div className={styles.chatBubbleUser}>How does ROS 2 handle real-time communication?</div>
                <div className={styles.chatBubbleBot}>
                  <span className={styles.typingIndicator}>
                    <span></span><span></span><span></span>
                  </span>
                </div>
              </div>
            </div>
          </div>

          {/* Personalization Card */}
          <div className={styles.aiCard}>
            <div className={styles.aiCardIcon}>
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                <circle cx="12" cy="12" r="3" />
                <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z" />
              </svg>
            </div>
            <h3 className={styles.aiCardTitle}>Adaptive Learning</h3>
            <p className={styles.aiCardDescription}>
              Content automatically adapts to your experience level and hardware setup.
            </p>
            <ul className={styles.aiCardFeatures}>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                Beginner hints
              </li>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                Advanced deep-dives
              </li>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                Hardware-specific tips
              </li>
            </ul>
          </div>

          {/* Translation Card */}
          <div className={styles.aiCard}>
            <div className={styles.aiCardIcon}>
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                <circle cx="12" cy="12" r="10" />
                <path d="M2 12h20M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z" />
              </svg>
            </div>
            <div className={styles.aiCardBadge}>اردو</div>
            <h3 className={styles.aiCardTitle}>Urdu Translation</h3>
            <p className={styles.aiCardDescription}>
              Access the full textbook in Urdu with AI-powered real-time translation.
            </p>
            <ul className={styles.aiCardFeatures}>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                AI-powered translation
              </li>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                Full RTL support
              </li>
              <li>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M20 6L9 17l-5-5" />
                </svg>
                Technical accuracy
              </li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}

function CTASection(): ReactNode {
  const { ref, isVisible } = useScrollAnimation();

  return (
    <section className={styles.cta} ref={ref}>
      <div className={styles.ctaBackground}>
        <div className={styles.ctaOrb1} />
        <div className={styles.ctaOrb2} />
      </div>
      <div className="container">
        <div className={clsx(styles.ctaContent, isVisible && styles.visible)}>
          <span className={styles.ctaEmoji}>🚀</span>
          <Heading as="h2" className={styles.ctaTitle}>
            Ready to Build the Future?
          </Heading>
          <p className={styles.ctaSubtitle}>
            Start your journey into Physical AI and Humanoid Robotics today.
            <br />
            Join the next generation of robotics engineers.
          </p>
          <div className={styles.ctaActions}>
            <Link
              className={clsx('button button--lg', styles.ctaPrimaryBtn)}
              to="/textbook/intro">
              <span className={styles.btnShine} />
              <span className={styles.btnContent}>
                Start Learning Now
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
                  <path d="M5 12h14M12 5l7 7-7 7" />
                </svg>
              </span>
            </Link>
            <Link
              className={clsx('button button--lg', styles.ctaSecondaryBtn)}
              to="/textbook/glossary">
              Browse Glossary
            </Link>
          </div>
          <div className={styles.ctaTrust}>
            <span>Built with</span>
            <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
              <path d="M12 21.35l-1.45-1.32C5.4 15.36 2 12.28 2 8.5 2 5.42 4.42 3 7.5 3c1.74 0 3.41.81 4.5 2.09C13.09 3.81 14.76 3 16.5 3 19.58 3 22 5.42 22 8.5c0 3.78-3.4 6.86-8.55 11.54L12 21.35z" />
            </svg>
            <span>by</span>
            <span className={styles.ctaTrustBrand}>Spec-Driven Development</span>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title="Home"
      description={siteConfig.tagline}>
      <main className={styles.main}>
        <HeroSection />
        <TechStackSection />
        <FeaturesSection />
        <ModulesSection />
        <AIFeaturesSection />
        <DeveloperSection />
        <CTASection />
      </main>
    </Layout>
  );
}
