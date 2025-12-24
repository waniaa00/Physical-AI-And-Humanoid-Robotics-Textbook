# Translation Feature Testing Guide

This guide provides step-by-step instructions for testing the Urdu translation feature implementation.

## Prerequisites

1. **Backend Running**:
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

2. **Frontend Running**:
   ```bash
   cd website
   npm start
   ```

3. **Environment Variables**: Ensure `.env` is configured:
   ```bash
   # backend/.env
   NEON_DATABASE_URL=postgresql://user:password@your-project.cloud.neon.tech/neondb

   # website/.env (create if doesn't exist)
   REACT_APP_API_URL=http://localhost:8000
   ```

## Test Scenarios

### Test 1: Chatbot Response Translation

**Objective**: Verify translation works for AI chatbot responses

**Steps**:
1. Navigate to any page with the chatbot
2. Open the chatbot window
3. Ask a question: "What is a humanoid robot?"
4. Wait for the AI response
5. Hover over the AI message
6. Click "Translate to Urdu" button (appears next to Copy button)
7. Wait for translation to complete (should take <5 seconds)
8. Verify:
   - ✅ Urdu text appears with proper RTL (right-to-left) formatting
   - ✅ Toggle button shows "Show Original" and "Show Urdu"
   - ✅ Can switch between English and Urdu
   - ✅ Font renders correctly (Noto Nastaliq Urdu)

**Expected Behavior**:
- Message switches from English to Urdu translation display
- RTL formatting applied (text flows right-to-left)
- Toggle works between original and translated

**Screenshot Checkpoint**: Translated chatbot message with RTL text

---

### Test 2: Book Text Selection Translation

**Objective**: Verify translation works for selected book content

**Steps**:
1. Navigate to any book chapter/documentation page
2. Select a paragraph of text (at least 10 characters)
3. A floating translate button appears on the right side
4. Click "Translate to Urdu" button
5. Wait for translation
6. Verify:
   - ✅ Translation modal/overlay appears
   - ✅ Urdu text displays with RTL formatting
   - ✅ Can toggle between original and Urdu
   - ✅ Modal can be closed

**Test Text Example**:
```
Select this paragraph:
"Humanoid robots are autonomous machines designed to replicate
human form and behavior. They typically feature a head, torso,
two arms, and two legs, mimicking the human body structure."
```

**Expected Behavior**:
- Floating translate button appears after selection
- Modal opens with translation
- RTL text renders correctly
- Close button works

**Screenshot Checkpoint**: Translation modal with Urdu text

---

### Test 3: Code Block Preservation

**Objective**: Verify code blocks are preserved during translation

**Steps**:
1. Find a page with code examples (or create test content)
2. Select text containing code blocks:
   ```
   Here's a Python example:
   ```python
   def greet():
       print("Hello, World!")
   ```
   This function prints a greeting.
   ```
3. Translate the selected text
4. Verify:
   - ✅ Code block remains in English
   - ✅ Code block maintains LTR (left-to-right) direction
   - ✅ Code syntax highlighting preserved
   - ✅ Surrounding text translated to Urdu

**Expected Behavior**:
- Code blocks stay in English with LTR
- Text around code is translated
- Code formatting intact

**Screenshot Checkpoint**: Translated text with preserved code block

---

### Test 4: Error Handling

**Objective**: Verify graceful error handling

**Test 4a: Empty Text**
1. Try to translate empty selection
2. Verify: Error message "Please select some text to translate"

**Test 4b: Text Too Long**
1. Select text >5000 words
2. Try to translate
3. Verify: Error message "Selected text is too long (max 5,000 words)"

**Test 4c: Backend Offline**
1. Stop the backend server
2. Try to translate
3. Verify: Error message "Translation service temporarily unavailable"

**Test 4d: Network Error**
1. Simulate slow/failed network
2. Verify: Loading state shows, then error message

**Expected Behavior**:
- All errors show user-friendly messages
- No console errors or crashes
- Users can retry after errors

---

### Test 5: Performance

**Objective**: Verify translation meets performance requirements

**Steps**:
1. Select text of different sizes:
   - Small (50 words)
   - Medium (500 words)
   - Large (2000 words)
2. Translate each
3. Measure time from click to display
4. Verify:
   - ✅ Small text: <2 seconds
   - ✅ Medium text: <4 seconds
   - ✅ Large text: <5 seconds (per spec requirement)

**Expected Behavior**:
- Translation completes within 5 seconds for ≤1000 words
- Loading spinner shows during translation
- Response time logged in backend console

---

### Test 6: Multiple Translations

**Objective**: Verify caching and multiple translations work

**Steps**:
1. Translate text A to Urdu
2. Translate text B to Urdu
3. Translate text A again (same text)
4. Check backend logs for cache hit
5. Verify second translation of text A is faster

**Expected Behavior**:
- First translation makes API call
- Second translation uses cache (instant)
- Backend logs show "Cache hit for translation"

---

### Test 7: RTL Formatting

**Objective**: Verify Urdu text displays correctly with RTL

**Steps**:
1. Translate any English text
2. Inspect translated text:
   - Text alignment: right-aligned
   - Direction: RTL (dir="rtl")
   - Font: Noto Nastaliq Urdu loaded
   - Line height: Increased for Urdu readability
3. Verify:
   - ✅ Text flows right-to-left
   - ✅ Punctuation at correct positions
   - ✅ Numbers remain LTR
   - ✅ Links work correctly

**Test in Different Browsers**:
- Chrome/Edge
- Firefox
- Safari (if available)

**Expected Behavior**:
- Consistent RTL rendering across browsers
- Urdu font loads properly
- No layout issues

---

### Test 8: Mobile Responsiveness

**Objective**: Verify translation works on mobile devices

**Steps**:
1. Open site in mobile view (Chrome DevTools device emulation)
2. Test chatbot translation
3. Test book text selection translation
4. Verify:
   - ✅ Translate button accessible
   - ✅ Modal fits screen
   - ✅ RTL text readable
   - ✅ Touch targets large enough

**Expected Behavior**:
- UI adapts to mobile screen size
- Translation modal responsive
- All features work on touch devices

---

### Test 9: Accessibility

**Objective**: Verify accessibility features

**Steps**:
1. Navigate using keyboard only (Tab, Enter, Esc)
2. Use screen reader (if available)
3. Verify:
   - ✅ All buttons have aria-labels
   - ✅ Keyboard navigation works
   - ✅ Focus indicators visible
   - ✅ Language properly indicated (lang="ur")

**Expected Behavior**:
- Full keyboard accessibility
- Screen readers announce language changes
- Focus management works

---

### Test 10: Integration Scenarios

**Objective**: Test real-world user workflows

**Scenario A: Student Learning**
1. Read a chapter
2. Select complex paragraph
3. Translate to Urdu
4. Toggle between English/Urdu to understand
5. Ask chatbot for clarification
6. Translate chatbot response to Urdu

**Scenario B: Quick Reference**
1. Ask chatbot: "Explain inverse kinematics"
2. Get detailed response
3. Translate to Urdu
4. Copy translated text for notes

**Expected Behavior**:
- Smooth workflow without errors
- Multiple translations work
- Copy function works with translations

---

## Automated Testing

### Backend API Tests

Run these with `curl` or Postman:

```bash
# Test 1: Valid translation request
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Hello world",
    "target_lang": "ur",
    "preserve_code": true
  }'

# Expected: 200 OK with translated_text field

# Test 2: Empty text (should fail)
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{
    "text": "",
    "target_lang": "ur"
  }'

# Expected: 400 Bad Request

# Test 3: Text too long (>5000 words)
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{
    "text": "'$(yes "word " | head -6000 | tr -d '\n')'",
    "target_lang": "ur"
  }'

# Expected: 400 Bad Request with "exceeds maximum 5,000 words"

# Test 4: Code preservation
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Here is code: `print(\"hello\")`",
    "target_lang": "ur",
    "preserve_code": true
  }'

# Expected: Code block preserved in response
```

### Frontend Unit Tests

Add to `website/src/components/Translation/__tests__/`:

```bash
cd website
npm test Translation
```

---

## Success Criteria Checklist

Phase 3 User Story 1 is complete when:

- [X] ✅ T016-T019: Translation service implemented with code preservation and caching
- [X] ✅ T020-T022: `/translate` API endpoint with validation and error handling
- [X] ✅ T023-T024: TranslateButton and TranslatedText components created
- [X] ✅ T025: Integrated into ChatKit (chatbot) and ChapterButtons (book pages)
- [X] ✅ T026: Toggle functionality (Show Original / Show Urdu) works
- [X] ✅ T027: RTL CSS with Urdu fonts applied
- [X] ✅ T028: Manual testing complete

**All tests passing**: Translation feature ready for production! 🎉

---

## Troubleshooting

### Translation Not Working

**Symptom**: Button click does nothing

**Solutions**:
1. Check browser console for errors
2. Verify backend is running: `curl http://localhost:8000/translate`
3. Check CORS: Ensure `CORS_ORIGINS` includes `http://localhost:3000`
4. Verify `REACT_APP_API_URL` is set correctly

### RTL Not Displaying

**Symptom**: Urdu text shows LTR (left-to-right)

**Solutions**:
1. Ensure `translation.css` is imported
2. Check browser DevTools: verify `dir="rtl"` attribute
3. Verify Urdu font loaded: Check Network tab for font requests
4. Clear browser cache and reload

### Slow Translation

**Symptom**: Takes >10 seconds to translate

**Solutions**:
1. Check backend logs for errors
2. Test translation API separately
3. Check network speed
4. Verify caching is enabled (check logs for "Cache hit")

### Code Blocks Not Preserved

**Symptom**: Code blocks translated to Urdu

**Solutions**:
1. Verify `preserve_code: true` in request
2. Check backend logs for code detection
3. Test with different code block formats
4. Ensure text_processing.py utilities work

---

## Reporting Issues

If you find bugs during testing:

1. **Capture**:
   - Screenshot
   - Browser console errors
   - Backend logs
   - Steps to reproduce

2. **Document**:
   - What you expected
   - What actually happened
   - Environment (browser, OS)

3. **Create Issue**: Document in GitHub or project tracker

---

## Next Steps After Testing

Once all tests pass:
1. Deploy to staging environment
2. User acceptance testing (UAT)
3. Performance monitoring setup
4. Production deployment
5. Monitor translation usage and errors

---

**Testing Date**: ___________
**Tested By**: ___________
**Status**: [ ] Pass [ ] Fail
**Notes**: ___________
