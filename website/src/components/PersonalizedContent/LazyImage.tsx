/**
 * LazyImage Component (T055)
 * Lazy-loaded image component with loading placeholder
 *
 * Features:
 * - Native lazy loading using loading="lazy" attribute
 * - Intersection Observer for advanced lazy loading
 * - Loading placeholder with smooth transition
 * - Error handling with fallback
 *
 * Usage:
 * <LazyImage src="/img/chapter-thumbnail.jpg" alt="Chapter thumbnail" />
 */

import React, { useState, useEffect, useRef } from 'react';
import styles from './LazyImage.module.css';

interface LazyImageProps {
  src: string;
  alt: string;
  className?: string;
  width?: number | string;
  height?: number | string;
  placeholder?: string;
}

export default function LazyImage({
  src,
  alt,
  className = '',
  width,
  height,
  placeholder = '/img/placeholder.svg',
}: LazyImageProps): React.JSX.Element {
  const [isLoaded, setIsLoaded] = useState(false);
  const [isInView, setIsInView] = useState(false);
  const [hasError, setHasError] = useState(false);
  const imgRef = useRef<HTMLImageElement>(null);

  // T055: Use Intersection Observer for advanced lazy loading
  useEffect(() => {
    if (!imgRef.current) return;

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            setIsInView(true);
            observer.disconnect();
          }
        });
      },
      {
        rootMargin: '50px', // Start loading 50px before image enters viewport
      }
    );

    observer.observe(imgRef.current);

    return () => observer.disconnect();
  }, []);

  const handleLoad = () => {
    setIsLoaded(true);
  };

  const handleError = () => {
    setHasError(true);
    setIsLoaded(true); // Show fallback
  };

  return (
    <div
      className={`${styles.container} ${className}`}
      style={{
        width: width ? `${width}px` : '100%',
        height: height ? `${height}px` : 'auto',
      }}
    >
      {/* Placeholder shown while image loads */}
      {!isLoaded && (
        <div className={styles.placeholder} aria-hidden="true">
          <div className={styles.spinner} />
        </div>
      )}

      {/* Actual image */}
      <img
        ref={imgRef}
        src={isInView ? src : placeholder}
        alt={alt}
        loading="lazy" // Native browser lazy loading
        className={`${styles.image} ${isLoaded ? styles.loaded : styles.loading}`}
        onLoad={handleLoad}
        onError={handleError}
        width={width}
        height={height}
        // Add decoding="async" for better performance
        decoding="async"
      />

      {/* Error fallback */}
      {hasError && (
        <div className={styles.error} role="img" aria-label={alt}>
          <span className={styles.errorIcon}>📷</span>
          <span className={styles.errorText}>Image unavailable</span>
        </div>
      )}
    </div>
  );
}
