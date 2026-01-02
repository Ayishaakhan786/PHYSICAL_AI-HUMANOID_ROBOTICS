import React from 'react';
import './SourceCitation.css';

interface SourceCitationProps {
  source: any;
}

/**
 * SourceCitation - Display a single source citation
 *
 * Shows a source with link, text preview, and relevance score.
 */
function SourceCitation({ source }: SourceCitationProps) {
  return (
    <div className="source-citation">
      <div className="source-citation-header">
        <a
          href={source.url}
          target="_blank"
          rel="noopener noreferrer"
          className="source-citation-link"
          title="Open in new tab"
        >
          📄 {source.section_path}
        </a>
        <span className={`source-citation-score source-citation-score-${getScoreLabel(source.relevance_score)}`}>
            {Math.round(source.relevance_score * 100)}% relevance
        </span>
      </div>
      <div className="source-citation-text">
        {source.text}
      </div>
    </div>
  );
}

/**
 * Get score label for CSS classes
 */
function getScoreLabel(score: number): string {
  if (score >= 0.75) return 'high';
  if (score >= 0.5) return 'medium';
  return 'low';
}

export default SourceCitation;
