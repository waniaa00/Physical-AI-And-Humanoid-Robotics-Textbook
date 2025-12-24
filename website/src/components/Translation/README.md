# Translation Components

This directory contains React components for translating book text and chatbot responses to Urdu with RTL (Right-to-Left) formatting.

## Components

### `TranslateButton`

Button component that triggers text translation to Urdu.

**Props:**
- `text` (string, required): Text to translate
- `onTranslate` (function, required): Callback when translation completes `(translatedText: string, originalText: string) => void`
- `className` (string, optional): Additional CSS classes
- `preserveCode` (boolean, optional, default: true): Whether to preserve code blocks during translation
- `userId` (string, optional): User ID for logging/analytics

**Example Usage:**
```tsx
import { TranslateButton, TranslatedText } from '@site/src/components/Translation';
import { useState } from 'react';

function MyBookPage() {
  const [translatedText, setTranslatedText] = useState<string | null>(null);
  const [originalText, setOriginalText] = useState<string>('');

  const selectedText = "This is the text from the book that the user selected.";

  const handleTranslate = (translated: string, original: string) => {
    setTranslatedText(translated);
    setOriginalText(original);
  };

  return (
    <div>
      <p>{selectedText}</p>

      <TranslateButton
        text={selectedText}
        onTranslate={handleTranslate}
      />

      {translatedText && (
        <TranslatedText
          originalText={originalText}
          translatedText={translatedText}
        />
      )}
    </div>
  );
}
```

### `TranslatedText`

Displays translated Urdu text with proper RTL formatting and toggle functionality.

**Props:**
- `originalText` (string, required): Original English text
- `translatedText` (string, required): Translated Urdu text
- `defaultShowTranslated` (boolean, optional, default: true): Whether to show translated text by default
- `className` (string, optional): Additional CSS classes

**Example Usage:**
```tsx
<TranslatedText
  originalText="Hello, world!"
  translatedText="ہیلو، دنیا!"
  defaultShowTranslated={true}
/>
```

## Integration Guide (T025)

### Option 1: Text Selection Integration

For book reading pages where users can select text:

```tsx
import React, { useState } from 'react';
import { TranslateButton, TranslatedText } from '@site/src/components/Translation';
import '@site/src/css/translation.css'; // Import RTL styles

function BookContent() {
  const [selectedText, setSelectedText] = useState('');
  const [translatedText, setTranslatedText] = useState<string | null>(null);
  const [originalText, setOriginalText] = useState('');

  // Handle text selection
  const handleTextSelection = () => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();
    if (text && text.length > 0) {
      setSelectedText(text);
    }
  };

  const handleTranslate = (translated: string, original: string) => {
    setTranslatedText(translated);
    setOriginalText(original);
  };

  return (
    <div onMouseUp={handleTextSelection}>
      <article className="book-content">
        {/* Your book content here */}
        <h1>Chapter 1: Introduction to Humanoid Robotics</h1>
        <p>Humanoid robots are designed to replicate human movements...</p>
      </article>

      {selectedText && (
        <div className="translation-controls">
          <TranslateButton
            text={selectedText}
            onTranslate={handleTranslate}
          />
        </div>
      )}

      {translatedText && (
        <TranslatedText
          originalText={originalText}
          translatedText={translatedText}
        />
      )}
    </div>
  );
}
```

### Option 2: Chatbot Integration

For translating chatbot responses:

```tsx
import { TranslateButton, TranslatedText } from '@site/src/components/Translation';

function ChatMessage({ message }) {
  const [translatedText, setTranslatedText] = useState<string | null>(null);

  const handleTranslate = (translated: string, original: string) => {
    setTranslatedText(translated);
  };

  return (
    <div className="chat-message">
      <div className="message-content">{message.text}</div>

      <TranslateButton
        text={message.text}
        onTranslate={handleTranslate}
        userId={message.userId}
      />

      {translatedText && (
        <TranslatedText
          originalText={message.text}
          translatedText={translatedText}
        />
      )}
    </div>
  );
}
```

### Option 3: Direct Component Usage

For pages where you want a dedicated translation section:

```tsx
import { TranslatedText } from '@site/src/components/Translation';
import '@site/src/css/translation.css';

function TranslatedChapter({ originalContent, urduContent }) {
  return (
    <div>
      <TranslatedText
        originalText={originalContent}
        translatedText={urduContent}
        defaultShowTranslated={false} // Show original by default
      />
    </div>
  );
}
```

## Testing (T028)

### Manual Testing Steps

1. **Start the backend server:**
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

2. **Start the frontend:**
   ```bash
   cd website
   npm start
   ```

3. **Test Translation Button:**
   - Navigate to a page with book content
   - Select some text (e.g., "Humanoid robots are autonomous machines designed to replicate human form and behavior.")
   - Click "Translate to Urdu" button
   - Verify:
     - Loading state appears ("Translating...")
     - Translation completes within 5 seconds
     - Urdu text displays with proper RTL formatting
     - Technical terms are preserved (e.g., "humanoid robots" should remain readable)

4. **Test Code Preservation:**
   - Select text containing code blocks:
     ```
     Here's a Python example:
     ```python
     def greet():
         print("Hello")
     ```
     Click translate to verify code blocks are preserved.
     ```
   - Verify code blocks remain in English with LTR formatting

5. **Test Toggle Functionality:**
   - After translation, click "Show Original" button
   - Verify text switches to English
   - Click "Show Urdu" button
   - Verify text switches back to Urdu with RTL formatting

6. **Test Error Handling:**
   - Try translating empty text (should show error)
   - Try translating very long text (>5000 words, should show error)
   - Turn off backend server and try translating (should show "Translation service temporarily unavailable")

### Automated Testing

Create test files in `website/src/components/Translation/__tests__/`:

```tsx
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { TranslateButton } from '../TranslateButton';
import { TranslatedText } from '../TranslatedText';

describe('TranslateButton', () => {
  it('renders translate button', () => {
    render(
      <TranslateButton
        text="Hello"
        onTranslate={jest.fn()}
      />
    );
    expect(screen.getByText('Translate to Urdu')).toBeInTheDocument();
  });

  it('shows loading state during translation', async () => {
    const onTranslate = jest.fn();
    render(
      <TranslateButton text="Hello" onTranslate={onTranslate} />
    );

    fireEvent.click(screen.getByText('Translate to Urdu'));

    await waitFor(() => {
      expect(screen.getByText('Translating...')).toBeInTheDocument();
    });
  });
});

describe('TranslatedText', () => {
  it('renders translated text with RTL', () => {
    render(
      <TranslatedText
        originalText="Hello"
        translatedText="ہیلو"
      />
    );
    expect(screen.getByText('ہیلو')).toBeInTheDocument();
  });

  it('toggles between original and translated', () => {
    render(
      <TranslatedText
        originalText="Hello"
        translatedText="ہیلو"
      />
    );

    fireEvent.click(screen.getByText('Show Original'));
    expect(screen.getByText('Hello')).toBeInTheDocument();

    fireEvent.click(screen.getByText('Show Urdu'));
    expect(screen.getByText('ہیلو')).toBeInTheDocument();
  });
});
```

## Configuration

### Environment Variables

Create or update `website/.env`:

```env
REACT_APP_API_URL=http://localhost:8000
```

For production:
```env
REACT_APP_API_URL=https://your-api-domain.com
```

### Backend Requirements

Ensure the backend translation API is running and accessible:
- Endpoint: `POST /translate`
- Request body: `{ text: string, target_lang: "ur", preserve_code: boolean, user_id?: string }`
- Response: `{ translated_text: string, original_text: string, ... }`

## Troubleshooting

### Translation Not Working

1. Check backend is running: `curl http://localhost:8000/translate`
2. Check CORS settings in `backend/main.py`
3. Check browser console for errors
4. Verify `REACT_APP_API_URL` is set correctly

### RTL Formatting Issues

1. Ensure `translation.css` is imported in your component
2. Check browser developer tools for CSS conflicts
3. Verify Urdu fonts are loading (check Network tab)

### Code Blocks Not Preserved

1. Check backend logs for code detection
2. Ensure `preserve_code: true` is passed to TranslateButton
3. Test with different code block formats (```, <code>, <pre>)

## Performance Optimization

- Translation results are cached on the backend for 24 hours
- Consider implementing frontend caching with localStorage
- Debounce translation requests for text selection
- Lazy load translation components for better initial page load

## Accessibility

- All buttons have proper `aria-label` attributes
- Keyboard navigation supported
- Error messages have `role="alert"`
- Language is properly indicated with `lang` and `dir` attributes
