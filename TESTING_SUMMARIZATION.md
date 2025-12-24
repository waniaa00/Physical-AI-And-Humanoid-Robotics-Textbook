# Summarization Feature Testing Guide

This document provides comprehensive test scenarios for the summarization feature integrated into the Humanoid Robotics documentation platform.

## Overview

The summarization feature allows users to:
- Get AI-powered summaries of chatbot responses
- Summarize selected text from documentation pages
- View compression metrics (word count, compression ratio, processing time)
- See extracted key points from summaries
- Toggle between summary and original text

## Prerequisites

1. **Backend Setup**:
   ```bash
   cd backend
   poetry install
   # Set environment variable
   export COHERE_API_KEY="your_cohere_api_key"
   poetry run uvicorn main:app --reload
   ```

2. **Frontend Setup**:
   ```bash
   cd website
   npm install
   npm start
   ```

3. **Verify API**:
   - Backend should be running at `http://localhost:8000`
   - Test endpoint: `http://localhost:8000/docs` (FastAPI Swagger UI)

## Test Scenarios

### Scenario 1: Summarize Chatbot Response (Short Text)

**Location**: Chat interface (`/docs/intro`)

**Steps**:
1. Navigate to any documentation page with the chatbot
2. Ask a question: "What is ROS 2?"
3. Wait for the chatbot response (should be 100-300 words)
4. Hover over the AI message to reveal action buttons
5. Click the "Summarize" button

**Expected Results**:
- ✅ Loading spinner appears during summarization
- ✅ Summary displays with metrics:
  - Original word count: ~150-200 words
  - Summary word count: ~30-40 words (20% compression for "short")
  - Compression ratio: ~20%
  - Processing time: 1-3 seconds
- ✅ Key points section appears (if available)
- ✅ Toggle button allows switching between summary and original text
- ✅ Summary is coherent and captures main concepts

**Test Cases**:
- TC1.1: Summary captures key concepts about ROS 2
- TC1.2: Code snippets (if any) are preserved correctly
- TC1.3: Metrics are accurate and match displayed content

---

### Scenario 2: Summarize Chatbot Response (Long Text)

**Location**: Chat interface

**Steps**:
1. Ask a complex question: "Explain forward kinematics and inverse kinematics with detailed examples"
2. Wait for detailed response (500+ words)
3. Hover over the AI message
4. Click the "Summarize" button

**Expected Results**:
- ✅ Summary compresses 500+ words to ~100 words (20% compression)
- ✅ Processing time is reasonable (2-5 seconds)
- ✅ Key points extracted include:
  - Definition of forward kinematics
  - Definition of inverse kinematics
  - Key differences
- ✅ Technical terminology is preserved
- ✅ Mathematical notation (if any) is preserved

**Test Cases**:
- TC2.1: Long responses (1000+ words) are handled correctly
- TC2.2: Complex technical content is summarized accurately
- TC2.3: Summary maintains technical accuracy

---

### Scenario 3: Summarize Selected Text from Documentation

**Location**: Book/documentation page (e.g., `/docs/book/chapter-1`)

**Steps**:
1. Navigate to a documentation page with substantial content
2. Select a paragraph or section (100-500 words)
3. Observe the floating action panel appears on the right
4. Click "Summarize" button in the panel

**Expected Results**:
- ✅ Floating panel shows selected text character count
- ✅ Both "Translate" and "Summarize" buttons are visible
- ✅ Loading state appears during summarization
- ✅ Modal opens with summary display:
  - Header: "Summary" in green theme
  - Metrics bar showing compression details
  - Summary text
  - Key points (if extracted)
  - "Show Original" toggle button
- ✅ Click outside modal closes it
- ✅ "Close" button dismisses modal

**Test Cases**:
- TC3.1: Short selections (50-100 words) are summarized appropriately
- TC3.2: Long selections (500+ words) are handled efficiently
- TC3.3: Code blocks within selected text are preserved
- TC3.4: Lists and bullet points are summarized into key points

---

### Scenario 4: Edge Cases - Text Too Short

**Location**: Chat interface or documentation page

**Steps**:
1. In chat, ask a very short question that gets a brief response (20-40 words)
2. Try to summarize the response

**Expected Results**:
- ✅ Error message appears: "Selected text is too short (minimum 50 words)"
- ✅ No API call is made
- ✅ Error is displayed in red alert box
- ✅ Button returns to enabled state after error

**Test Cases**:
- TC4.1: Text with 49 words shows error
- TC4.2: Text with exactly 50 words allows summarization
- TC4.3: Error message is clear and actionable

---

### Scenario 5: Edge Cases - Text Too Long

**Location**: Documentation page

**Steps**:
1. Select a very large section of text (5000+ words)
2. Try to summarize

**Expected Results**:
- ✅ Error message appears: "Selected text is too long (max 5,000 words)"
- ✅ No API call is made
- ✅ Clear error feedback provided

**Test Cases**:
- TC5.1: Text with 5001 words shows error
- TC5.2: Text with exactly 5000 words allows summarization

---

### Scenario 6: Code Block Preservation

**Location**: Chat or documentation with code examples

**Steps**:
1. Ask chatbot: "Show me a Python example for ROS 2 publisher"
2. Receive response with code block
3. Summarize the response

**Expected Results**:
- ✅ Summary explains the concept in natural language
- ✅ Code block is preserved exactly as-is
- ✅ Code syntax is not translated or modified
- ✅ Summary focuses on "concepts" (default focus setting)

**Test Cases**:
- TC6.1: Inline code (`code`) is preserved
- TC6.2: Code blocks (```python```) are preserved
- TC6.3: Multiple code blocks are all preserved
- TC6.4: Code comments are preserved

---

### Scenario 7: Metrics Accuracy

**Location**: Any summarization instance

**Steps**:
1. Summarize a known piece of text
2. Verify all metrics displayed

**Expected Results**:
- ✅ **Original Word Count**: Matches actual word count (manually counted)
- ✅ **Summary Word Count**: Matches summary length
- ✅ **Compression Ratio**: Calculated as (summary_words / original_words) × 100
- ✅ **Processing Time**: Displayed in seconds with 1 decimal place
- ✅ All metrics update correctly when toggling between summary and original

**Test Cases**:
- TC7.1: 200-word text shows correct metrics
- TC7.2: Compression ratio is approximately 20% for "short" target
- TC7.3: Processing time is reasonable (1-5 seconds for typical text)

---

### Scenario 8: Key Points Extraction

**Location**: Any summarization with structured content

**Steps**:
1. Ask chatbot: "What are the main components of a humanoid robot?"
2. Receive a response with clear bullet points or numbered list
3. Summarize the response

**Expected Results**:
- ✅ Key points section appears below summary
- ✅ Each key point is displayed in its own styled box
- ✅ Key points are extracted from bullet points or main concepts
- ✅ Typically 3-5 key points are shown
- ✅ Key points are concise (1-2 sentences each)

**Test Cases**:
- TC8.1: Responses with explicit lists generate key points
- TC8.2: Narrative text generates extracted key points
- TC8.3: Key points are distinct and non-redundant

---

### Scenario 9: Toggle Between Summary and Original

**Location**: Any summarization instance

**Steps**:
1. Summarize any chatbot response or selected text
2. Click "Show Original" button
3. Click "Show Summary" button
4. Repeat toggle multiple times

**Expected Results**:
- ✅ Toggle button label changes:
  - When showing summary: "Show Original" with 🔙 icon
  - When showing original: "Show Summary" with 📝 icon
- ✅ Content switches smoothly without flicker
- ✅ Metrics bar remains visible and unchanged
- ✅ Key points section only visible when showing summary
- ✅ State persists correctly on each toggle

**Test Cases**:
- TC9.1: Toggle works on chatbot messages
- TC9.2: Toggle works on documentation selection modal
- TC9.3: Multiple toggles don't cause errors

---

### Scenario 10: Error Handling - API Failure

**Location**: Any summarization instance

**Steps**:
1. Stop the backend server
2. Try to summarize text
3. Restart the backend server
4. Try to summarize again

**Expected Results**:
- ✅ When backend is down:
  - Error message appears: "Summarization failed"
  - Error is logged to console
  - Button returns to enabled state
  - User can retry after fixing issue
- ✅ When backend is restored:
  - Summarization works correctly
  - No lingering errors

**Test Cases**:
- TC10.1: Network timeout handled gracefully
- TC10.2: 500 Internal Server Error shows user-friendly message
- TC10.3: 400 Bad Request shows validation error details
- TC10.4: Retry mechanism works after error

---

### Scenario 11: Cohere API Integration

**Location**: Backend testing

**Steps**:
1. Use invalid Cohere API key
2. Try to summarize text
3. Use valid Cohere API key
4. Try to summarize again

**Expected Results**:
- ✅ Invalid API key results in proper error message
- ✅ Valid API key enables successful summarization
- ✅ API responses are cached for identical requests
- ✅ Different target lengths produce different summaries

**Test Cases**:
- TC11.1: "short" target length produces ~20% compression
- TC11.2: "medium" target length produces ~30% compression (if implemented)
- TC11.3: "long" target length produces ~40% compression (if implemented)
- TC11.4: Caching reduces API calls for repeated requests

---

### Scenario 12: Dark Mode Compatibility

**Location**: Any page with dark mode enabled

**Steps**:
1. Enable dark mode in browser/system settings
2. Summarize chatbot response
3. Summarize selected text from documentation

**Expected Results**:
- ✅ Summary display adapts to dark theme:
  - Background: Dark gray (#1f2937)
  - Text: Light gray (#f9fafb)
  - Metrics bar: Dark green (#064e3b)
  - Borders: Darker tones
- ✅ All text remains readable
- ✅ Icons and buttons have proper contrast
- ✅ Modal overlays use dark theme

**Test Cases**:
- TC12.1: Summary display in dark mode is readable
- TC12.2: Metrics bar has sufficient contrast
- TC12.3: Key points section is styled appropriately

---

### Scenario 13: Responsive Design

**Location**: Any page (test on mobile viewport)

**Steps**:
1. Resize browser to mobile size (375px width)
2. Open chatbot and get a response
3. Try to summarize
4. Select text from documentation
5. Try to summarize selected text

**Expected Results**:
- ✅ Chatbot interface:
  - Buttons stack vertically on small screens
  - Summary display is readable on narrow viewport
  - Metrics wrap to multiple lines if needed
- ✅ Documentation page:
  - Floating action panel adjusts position
  - Modal is responsive and scrollable
  - Summary display adapts to mobile

**Test Cases**:
- TC13.1: Mobile viewport (375px) is usable
- TC13.2: Tablet viewport (768px) is usable
- TC13.3: Desktop viewport (1024px+) uses optimal layout

---

## API Testing

### Direct API Tests (using curl or Postman)

**Test 1: Valid Summarization Request**
```bash
curl -X POST "http://localhost:8000/summarize" \
  -H "Content-Type: application/json" \
  -d '{
    "text": "ROS 2 is the next generation of the Robot Operating System. It provides a standard software platform for robot developers across industries. ROS 2 builds upon the success of ROS 1 while addressing its limitations. It offers improved real-time capabilities, better security, and multi-robot support. The architecture is designed for production systems and includes Quality of Service settings.",
    "target_length": "short",
    "focus": "concepts"
  }'
```

**Expected Response**:
```json
{
  "summary": "ROS 2 is the improved successor to ROS 1, offering a standard platform with enhanced real-time performance, security, and multi-robot capabilities for production systems.",
  "original_word_count": 58,
  "summary_word_count": 11,
  "compression_ratio": 0.19,
  "processing_time_ms": 1234,
  "key_points": [
    "Next generation of Robot Operating System",
    "Improved real-time, security, and multi-robot support",
    "Designed for production systems"
  ]
}
```

**Test 2: Text Too Short**
```bash
curl -X POST "http://localhost:8000/summarize" \
  -H "Content-Type: application/json" \
  -d '{
    "text": "ROS 2 is great.",
    "target_length": "short",
    "focus": "concepts"
  }'
```

**Expected Response** (422 Validation Error):
```json
{
  "detail": {
    "message": "Text must be at least 50 words"
  }
}
```

**Test 3: Invalid Target Length**
```bash
curl -X POST "http://localhost:8000/summarize" \
  -H "Content-Type: application/json" \
  -d '{
    "text": "...",
    "target_length": "invalid",
    "focus": "concepts"
  }'
```

**Expected Response** (422 Validation Error):
```json
{
  "detail": [
    {
      "loc": ["body", "target_length"],
      "msg": "value is not a valid enumeration member; permitted: 'short', 'medium', 'long'",
      "type": "type_error.enum"
    }
  ]
}
```

---

## Performance Benchmarks

| Text Length (words) | Target Length | Expected Processing Time | Expected Compression |
|---------------------|---------------|-------------------------|---------------------|
| 100                 | short         | 1-2 seconds            | ~20 words           |
| 500                 | short         | 2-3 seconds            | ~100 words          |
| 1000                | short         | 3-4 seconds            | ~200 words          |
| 5000                | short         | 5-8 seconds            | ~1000 words         |

---

## Known Issues & Limitations

1. **Cohere API Rate Limits**: Free tier may have rate limits; test accordingly
2. **Maximum Text Length**: 5,000 words to prevent API timeouts
3. **Minimum Text Length**: 50 words to ensure meaningful summaries
4. **Caching**: In-memory cache clears on server restart
5. **Focus Parameter**: Currently only "concepts" is fully optimized

---

## Success Criteria

- ✅ All 13 scenarios pass
- ✅ API tests return expected responses
- ✅ Performance benchmarks are met
- ✅ No console errors or warnings
- ✅ Dark mode and responsive design work correctly
- ✅ Error handling is robust and user-friendly
- ✅ Summaries are accurate and coherent
- ✅ Metrics are calculated correctly
- ✅ Code blocks are preserved

---

## Next Steps

After successful testing:
1. Document any bugs found in GitHub Issues
2. Optimize summarization prompts for better quality
3. Add user feedback mechanism
4. Implement summary rating system
5. Add support for "medium" and "long" target lengths
6. Implement persistent caching (Redis)
7. Add summarization history per user
8. Integrate with user interests for personalized summaries (Phase 6)
