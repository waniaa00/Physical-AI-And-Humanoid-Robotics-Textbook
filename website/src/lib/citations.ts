/**
 * Citation parsing utilities
 * Extract inline citations and source URLs from agent responses
 */
import type { SourceCitation } from '../types/chat';

export interface ParsedCitations {
  inlineCitations: Array<{ number: number; placeholder: string }>;
  sources: SourceCitation[];
}

/**
 * Parse citations from agent response text
 * Extracts both inline [Source N] markers and source URLs from the Sources: section
 */
export function parseCitations(response: string): ParsedCitations {
  // Extract inline citations: [Source 1], [Source 2], etc.
  const citationRegex = /\[Source (\d+)\]/g;
  const inlineCitations = Array.from(response.matchAll(citationRegex)).map(match => ({
    number: parseInt(match[1], 10),
    placeholder: match[0],
  }));

  // Extract source URLs from "Sources:" section
  const sources: SourceCitation[] = [];

  // Find the "Sources:" section
  const sourcesMatch = response.match(/Sources?:\s*\n([\s\S]*?)(?:\n\n|\n*$)/i);

  if (sourcesMatch) {
    const sourcesText = sourcesMatch[1];

    // Match patterns like:
    // - [Source 1] https://example.com
    // - [Source 1]: https://example.com
    // [Source 1] https://example.com
    const sourceLineRegex = /\[Source (\d+)\]:?\s*(https?:\/\/[^\s]+)/gi;

    const sourceMatches = Array.from(sourcesText.matchAll(sourceLineRegex));

    for (const match of sourceMatches) {
      const number = parseInt(match[1], 10);
      const url = match[2].trim();

      sources.push({
        number,
        url,
      });
    }
  }

  return {
    inlineCitations,
    sources,
  };
}

/**
 * Extract text snippet from response for a given citation number
 * Returns the sentence or paragraph containing the citation
 */
export function getCitationContext(response: string, citationNumber: number): string | null {
  const placeholder = `[Source ${citationNumber}]`;
  const index = response.indexOf(placeholder);

  if (index === -1) return null;

  // Find the sentence containing this citation
  // Look backwards for sentence start
  let start = index;
  while (start > 0 && !response[start].match(/[.!?]\s/)) {
    start--;
  }
  if (start > 0) start += 2; // Skip the punctuation and space

  // Look forwards for sentence end
  let end = index;
  while (end < response.length && !response[end].match(/[.!?]/)) {
    end++;
  }
  if (end < response.length) end++; // Include the punctuation

  return response.substring(start, end).trim();
}

/**
 * Check if response contains any citations
 */
export function hasCitations(response: string): boolean {
  return /\[Source \d+\]/.test(response);
}

/**
 * Get all unique citation numbers from response
 */
export function getCitationNumbers(response: string): number[] {
  const matches = Array.from(response.matchAll(/\[Source (\d+)\]/g));
  const numbers = matches.map(m => parseInt(m[1], 10));
  return Array.from(new Set(numbers)).sort((a, b) => a - b);
}
