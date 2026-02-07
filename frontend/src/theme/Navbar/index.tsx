import React, { JSX, useState, useEffect } from 'react';
import Navbar from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type { WrapperProps } from '@docusaurus/types';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;

  const [user, setUser] = useState<any>(() => {
    if (typeof window !== 'undefined') {
      const userData = localStorage.getItem('user');
      return userData ? JSON.parse(userData) : null;
    }
    return null;
  });

  const [loading, setLoading] = useState(false);

  useEffect(() => {
    if (typeof document !== 'undefined') {
      const hideAuthLinks = () => {
        if (user) {
          document.documentElement.classList.add('user-authenticated');
          document.body.classList.add('user-authenticated');
        } else {
          document.documentElement.classList.remove('user-authenticated');
          document.body.classList.remove('user-authenticated');
        }

        // Failsafe: Hide via direct DOM manipulation
        const authLinks = document.querySelectorAll('a[href*="signup"], a[href*="signin"], .navbar__item:has(a[href*="signup"]), .navbar__item:has(a[href*="signin"])');
        authLinks.forEach(link => {
          if (user) {
            (link as HTMLElement).style.setProperty('display', 'none', 'important');
          } else {
            (link as HTMLElement).style.setProperty('display', '', '');
          }
        });
      };

      hideAuthLinks();
      // Run again after a delay to catch late Docusaurus renders
      const timer = setTimeout(hideAuthLinks, 500);
      const timer2 = setTimeout(hideAuthLinks, 2000);
      return () => {
        clearTimeout(timer);
        clearTimeout(timer2);
      };
    }
  }, [user]);

  const handleLogout = () => {
    localStorage.removeItem('user');
    localStorage.removeItem('token');
    setUser(null);
    window.location.href = baseUrl;
  };

  const handlePersonalize = async () => {
    if (!user) {
      alert('Please sign in to personalize content');
      window.location.href = `${baseUrl}signin`;
      return;
    }

    setLoading(true);
    try {
      const contentElement = document.querySelector('.markdown');
      if (!contentElement) {
        alert('No content found to personalize');
        return;
      }

      const pageContent = contentElement.textContent || '';

      const API_URL = process.env.NODE_ENV === 'production'
        ? 'https://areeba-fatima-book.hf.space/api'
        : 'http://localhost:8000/api';

      const token = localStorage.getItem('token');
      const response = await fetch(`${API_URL}/personalize/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({
          content: pageContent,
          software_experience: user.software_experience,
          hardware_experience: user.hardware_experience
        })
      });

      if (!response.ok) throw new Error('Personalization failed');

      const data = await response.json();
      contentElement.innerHTML = data.personalized_content;
      alert('✨ Content personalized for your experience level!');
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Personalization failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleTranslate = async () => {
    setLoading(true);
    try {
      // Target the main content area (documentation, chatbot, or general page content)
      const contentElement =
        document.querySelector('.markdown') ||
        document.querySelector('article') ||
        document.querySelector('[class*="chatContainer"]') ||
        document.querySelector('main');

      if (!contentElement) {
        alert('No translatable content found on this page.');
        return;
      }

      const pageContent = contentElement.textContent || '';

      const API_URL = process.env.NODE_ENV === 'production'
        ? 'https://areeba-fatima-book.hf.space/api'
        : 'http://localhost:8000/api';

      const response = await fetch(`${API_URL}/translate/urdu`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content: pageContent,
          target_lang: 'ur',
          preserve_code: true
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      contentElement.innerHTML = data.translated;
      contentElement.style.direction = 'rtl';
      contentElement.style.textAlign = 'right';
      alert('🌐 Content translated to Urdu!');
    } catch (error) {
      console.error('Translation error:', error);
      alert('Translation failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <>
      <Navbar {...props} />
      {user && (
        <div className={styles.headerAuth}>
          <div className={styles.userInfo}>
            <span className={styles.userName}>
              👤 {user.name || user.username}
            </span>
            <button
              onClick={handleLogout}
              className={styles.logoutBtn}
            >
              Sign Out
            </button>
          </div>
        </div>
      )}
      <div className={`${styles.navbarControls} ${user ? styles.authenticated : ''}`}>
        <button
          onClick={handlePersonalize}
          disabled={loading}
          className={styles.personalizeBtn}
          title="Personalize content based on your experience level"
        >
          {loading ? '⏳' : '⚡ Personalize'}
        </button>
        <button
          onClick={handleTranslate}
          disabled={loading}
          className={styles.translateBtn}
          title="Translate to Urdu"
        >
          {loading ? '⏳' : '🌐 اردو'}
        </button>
      </div>
    </>
  );
}
