import { useState, useEffect } from 'react';
import { useDoc } from '@docusaurus/plugin-content-docs/client';

/**
 * Custom hook to extract current chapter ID from Docusaurus metadata
 * @returns Current chapter ID or null if not available
 */
export const useCurrentChapter = () => {
  const { metadata } = useDoc();
  const [chapterId, setChapterId] = useState<string | null>(null);

  useEffect(() => {
    if (metadata && metadata.id) {
      setChapterId(metadata.id);
    }
  }, [metadata]);

  return chapterId;
};