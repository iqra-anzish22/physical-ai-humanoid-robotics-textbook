import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './TranslationToggle.module.css';

// TranslationToggle component to allow users to switch between English and Urdu content
const TranslationToggle = ({ availableLanguages = ['en'], currentLanguage = 'en', onLanguageChange }) => {
  const [selectedLanguage, setSelectedLanguage] = useState(currentLanguage);

  useEffect(() => {
    setSelectedLanguage(currentLanguage);
  }, [currentLanguage]);

  const handleLanguageChange = (langCode) => {
    setSelectedLanguage(langCode);
    if (onLanguageChange) {
      onLanguageChange(langCode);
    }
  };

  // Language name mapping
  const getLanguageName = (code) => {
    const languageNames = {
      'en': 'English',
      'ur': 'Urdu'
    };
    return languageNames[code] || code.toUpperCase();
  };

  if (!availableLanguages || availableLanguages.length <= 1) {
    return null; // Don't show toggle if only one language is available
  }

  return (
    <div className={clsx('translation-toggle-container', styles.container)}>
      <div className={styles.dropdown}>
        <select
          className={styles.select}
          value={selectedLanguage}
          onChange={(e) => handleLanguageChange(e.target.value)}
          aria-label="Select language"
        >
          {availableLanguages.map((langCode) => (
            <option key={langCode} value={langCode}>
              {getLanguageName(langCode)}
            </option>
          ))}
        </select>
        <div className={styles.selectArrow}></div>
      </div>
    </div>
  );
};

export default TranslationToggle;