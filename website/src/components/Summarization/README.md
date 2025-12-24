# Summarization Components

This directory contains React components for AI-powered text summarization functionality in the Humanoid Robotics documentation platform.

## Components

### SummarizeButton

A button component that triggers text summarization via the backend API.

**Props:**
- `text` (string, required): The text to summarize
- `onSummarize` (function, required): Callback when summarization completes, receives `(summary, originalText, metrics)`
- `targetLength` ('short' | 'medium' | 'long', optional): Target summary length (default: 'short')
  - `short`: ~20% of original text
  - `medium`: ~30% of original text
  - `long`: ~40% of original text
- `focus` ('concepts' | 'code' | 'both', optional): Summary focus (default: 'concepts')
- `userId` (string, optional): User ID for logging/analytics
- `className` (string, optional): Additional CSS classes

**Features:**
- Loading state with spinner animation
- Input validation (50-5000 words)
- Error handling with user-friendly messages
- Configurable summarization parameters
- Calls `/summarize` API endpoint

**Usage:**
```tsx
import { SummarizeButton } from '@site/src/components/Summarization';

<SummarizeButton
  text={selectedText}
  onSummarize={(summary, original, metrics) => {
    console.log('Summary:', summary);
    console.log('Metrics:', metrics);
  }}
  targetLength="short"
  focus="concepts"
/>
```

### SummaryDisplay

A display component for showing summarized text with metrics and key points.

**Props:**
- `summary` (string, required): The generated summary text
- `originalText` (string, required): The original text before summarization
- `metrics` (SummaryMetrics, required): Summarization metrics object
  - `original_word_count`: Number of words in original text
  - `summary_word_count`: Number of words in summary
  - `compression_ratio`: Compression ratio (0-1, e.g., 0.20 = 20%)
  - `processing_time_ms`: Time taken to generate summary (milliseconds)
  - `key_points?`: Optional array of extracted key points
- `defaultShowSummary` (boolean, optional): Show summary by default (default: true)
- `className` (string, optional): Additional CSS classes

**Features:**
- Toggle between summary and original text
- Metrics bar showing compression statistics
- Key points section (if available)
- Responsive design with mobile support
- Dark mode support
- Emoji icons for visual clarity

**Usage:**
```tsx
import { SummaryDisplay } from '@site/src/components/Summarization';

<SummaryDisplay
  summary="ROS 2 is the improved successor to ROS 1..."
  originalText="ROS 2 is the next generation of the Robot Operating System..."
  metrics={{
    original_word_count: 150,
    summary_word_count: 30,
    compression_ratio: 0.20,
    processing_time_ms: 1234,
    key_points: [
      'Next generation of Robot Operating System',
      'Improved real-time capabilities'
    ]
  }}
  defaultShowSummary={true}
/>
```

## Integration Examples

### ChatKit (Chatbot Messages)

Integrated into `MessageList.tsx` to allow users to summarize AI responses:

```tsx
const [summarizedMessages, setSummarizedMessages] = useState<Map<string, SummarizedMessage>>(new Map());

const handleSummarize = (messageId: string) => (
  summary: string,
  originalText: string,
  metrics: SummaryMetrics
) => {
  setSummarizedMessages(new Map(summarizedMessages.set(messageId, {
    messageId,
    summary,
    originalText,
    metrics
  })));
};

// In render:
{isMessageSummarized(message.id) ? (
  <SummaryDisplay
    summary={summarizedMessages.get(message.id)!.summary}
    originalText={summarizedMessages.get(message.id)!.originalText}
    metrics={summarizedMessages.get(message.id)!.metrics}
  />
) : (
  <div>{message.content}</div>
)}
```

### ChapterButtons (Documentation Pages)

Integrated into `ChapterButtons/index.tsx` for selected text summarization:

```tsx
const [summaryText, setSummaryText] = useState<string | null>(null);
const [summaryMetrics, setSummaryMetrics] = useState<SummaryMetrics | null>(null);

const handleSummarize = (summary: string, original: string, metrics: SummaryMetrics) => {
  setSummaryText(summary);
  setSummaryMetrics(metrics);
  setShowActionButtons(false);
};

// Floating action panel
<SummarizeButton
  text={selectedText}
  onSummarize={handleSummarize}
  targetLength="short"
  focus="concepts"
/>

// Summary modal
{summaryText && summaryMetrics && (
  <div className="modal">
    <SummaryDisplay
      summary={summaryText}
      originalText={originalText}
      metrics={summaryMetrics}
    />
  </div>
)}
```

## API Endpoint

The components interact with the `/summarize` endpoint:

**Request:**
```json
{
  "text": "Text to summarize...",
  "target_length": "short",
  "focus": "concepts",
  "user_id": "optional-user-id"
}
```

**Response:**
```json
{
  "summary": "Summarized text...",
  "original_word_count": 150,
  "summary_word_count": 30,
  "compression_ratio": 0.20,
  "processing_time_ms": 1234,
  "key_points": [
    "Key point 1",
    "Key point 2"
  ]
}
```

## Styling

Components use CSS Modules for scoped styling:
- `SummarizeButton.module.css`: Button styles with loading states
- `SummaryDisplay.module.css`: Display styles with metrics bar and key points

Both support:
- Light/dark mode via `@media (prefers-color-scheme: dark)`
- Responsive design via breakpoints
- Smooth transitions and animations

## Environment Variables

Set `REACT_APP_API_URL` to configure the backend URL (default: `http://localhost:8000`):

```bash
# .env.local
REACT_APP_API_URL=http://localhost:8000
```

## Testing

See [TESTING_SUMMARIZATION.md](../../../../TESTING_SUMMARIZATION.md) for comprehensive test scenarios covering:
- Chatbot message summarization
- Documentation text selection
- Edge cases (too short, too long)
- Code block preservation
- Metrics accuracy
- Error handling
- Dark mode and responsive design

## Implementation Details

### Word Count Validation
- Minimum: 50 words
- Maximum: 5,000 words
- Validation happens on both frontend and backend

### Compression Targets
- **Short**: ~20% of original (default)
- **Medium**: ~30% of original
- **Long**: ~40% of original

### Focus Modes
- **Concepts**: Summarize main ideas and concepts (default)
- **Code**: Focus on code examples and technical details
- **Both**: Balance between concepts and code

### Key Points Extraction
- Automatically extracts 3-5 key points from summaries
- Detects bullet points and main concepts
- Displayed in styled boxes below summary

## Dependencies

- React 18+
- TypeScript
- Cohere LLM (backend)
- FastAPI (backend)

## Future Enhancements

1. Summary quality rating system
2. Persistent summary history per user
3. Advanced focus modes (code-only, theory-only)
4. Multi-language summarization
5. Personalized summaries based on user interests
6. Summary export (PDF, Markdown)
7. Collaborative summary editing
