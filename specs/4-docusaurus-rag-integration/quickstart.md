# Quickstart Guide: Chat with the Humanoid Robotics Book

**Feature**: Docusaurus RAG Frontend Integration
**Version**: 1.0.0
**Last Updated**: 2025-12-15

## What is this?

The Humanoid Robotics book now includes an interactive chat assistant that can answer questions about the book content. Simply click the chat button in the bottom-right corner of any page to start asking questions.

The assistant uses the book's content to provide accurate, grounded answers with source citations, so you can verify the information and explore further.

---

## Getting Started

### Opening the Chat

1. **Look for the chat button** in the bottom-right corner of any book page (floating blue button with a message icon)
2. **Click the button** to open the chat window
3. **Type your question** in the input field at the bottom
4. **Press Enter or click Send** to submit your question
5. **Wait for the response** - the assistant will "think" for a few seconds and then provide an answer

### Example Questions

Try asking:
- "What is inverse kinematics?"
- "Explain the zero-moment point"
- "How does bipedal walking work?"
- "What are the main challenges in humanoid robotics?"

---

## Features

### 1. Ask Questions About the Book

Ask any question related to humanoid robotics topics covered in the book. The assistant will search through the book content and provide an answer with citations to the relevant sections.

**How it works:**
- Type your question in the chat input
- The assistant searches the book content
- You receive an answer with source citations (e.g., [Source 1], [Source 2])
- Click on citations to jump to the relevant book sections

**Example Conversation:**

*You:* "What is inverse kinematics?"

*Assistant:* "Inverse kinematics (IK) is the process of calculating the joint angles required to position a robotic end-effector at a desired location [Source 1]. Unlike forward kinematics which calculates position from angles, IK solves the reverse problem [Source 2].

Sources:
- [Source 1] /docs/kinematics/inverse-kinematics
- [Source 2] /docs/kinematics/forward-vs-inverse"

---

### 2. Ask About Selected Text

If you're reading a specific passage and want clarification, you can select the text and ask the assistant to explain it in simpler terms.

**How it works:**
1. **Select text** by clicking and dragging across any paragraph or passage
2. **Click "Ask about this"** button that appears near your selection
3. **Ask your question** (e.g., "Explain this in simpler terms")
4. The assistant will answer based ONLY on the selected text (no broader search)

**Example:**

*You select:* "The zero-moment point (ZMP) is the point on the ground where the sum of all moments equals zero."

*You ask:* "Explain this in simpler terms"

*Assistant:* "The zero-moment point (ZMP) is essentially the spot on the ground where a robot is perfectly balanced. Think of it like the center of a seesaw - when the seesaw is balanced, the forces on both sides cancel out. For a walking robot, the ZMP is the point where all the twisting forces (moments) from the robot's weight and movement balance out to zero. If the ZMP stays inside the robot's footprint, the robot won't tip over."

**Notice:** When asking about selected text, you'll see a badge that says "Based on selection" at the top of the chat window.

---

### 3. Multi-Turn Conversations

You can have a back-and-forth conversation with the assistant, asking follow-up questions without needing to repeat context.

**Example Conversation:**

*You:* "What is forward kinematics?"

*Assistant:* "Forward kinematics (FK) is the process of calculating the position and orientation of a robot's end-effector based on given joint angles [Source 1]..."

*You:* "How does it differ from inverse kinematics?"

*Assistant:* "The key difference is the direction of calculation: Forward Kinematics (FK) calculates end-effector position FROM joint angles, while Inverse Kinematics (IK) calculates joint angles FROM desired position [Source 1]..."

**Notice:** The assistant remembers the context of your conversation, so "it" refers to forward kinematics in the second question.

---

### 4. Source Citations

Every answer includes citations to the book sections where the information was found. This allows you to:
- Verify the accuracy of the answer
- Explore the topic in more depth
- Navigate directly to relevant sections

**How to use citations:**
- Look for **[Source 1]**, **[Source 2]**, etc. in the answer text
- Scroll to the bottom of the answer to see the **Sources:** list
- **Click on any source** to open the book section in a new tab

---

## Tips for Best Results

### Do:
✅ Ask specific questions about robotics topics covered in the book
✅ Use clear, concise language
✅ Ask follow-up questions to dive deeper
✅ Select text when you need clarification on a specific passage
✅ Click on citations to verify information and learn more

### Don't:
❌ Ask off-topic questions (e.g., "What's the weather?") - the assistant only answers questions about the book content
❌ Ask about topics not covered in the book - the assistant cannot answer questions beyond the book's scope
❌ Expect the assistant to remember conversations from previous browser sessions - each tab has its own conversation that expires when you close the tab

---

## Understanding Different Response Types

### Success Response
Normal answer with citations. This is what you'll see most of the time.

**Example:**
"Inverse kinematics solves for joint angles [Source 1]..."

### Context-Constrained Response
When you ask about selected text, the answer will be based ONLY on that text (no retrieval from the book).

**How to tell:**
- Look for the "Based on selection" badge in the chat header
- The answer explains the selected passage without citing other sources

### Off-Topic Response
If you ask a question outside the book's scope, you'll see a guardrail message.

**Example:**
"I can only answer questions about humanoid robotics based on the book content. Please ask a question related to robotics topics like kinematics, balance, gait, or control systems."

---

## Managing Your Conversation

### Starting a New Conversation

1. Click the **refresh icon** (↻) in the chat header
2. This clears your current conversation and starts fresh
3. Useful when switching to a completely different topic

### Closing the Chat

- Click the **X button** in the chat header
- Or click the **chat button** in the bottom-right corner again
- Your conversation is saved and will reappear when you reopen the chat (within the same browser tab)

### Switching Between Full-Book and Selected-Text Mode

**To use Selected-Text Mode:**
1. Select text on the page
2. Click "Ask about this"
3. You'll see a "Based on selection" badge and a preview of the selected text

**To return to Full-Book Mode:**
1. Click the **"Clear"** button next to the selected text preview
2. The chat will switch back to searching the full book

---

## Troubleshooting

### The chat button doesn't appear
- **Check your browser:** Make sure you're using a modern browser (Chrome, Firefox, Safari, or Edge)
- **Refresh the page:** Try reloading the page (Ctrl+R or Cmd+R)
- **Check browser console:** Open developer tools (F12) and look for errors

### "Unable to connect to server" error
- **Check your internet connection:** Make sure you're online
- **Try refreshing the page:** The backend service might be temporarily unavailable
- **Click "Retry":** The error message should include a Retry button

### "Request timed out" error
- **Your question might be complex:** Some questions take longer to answer
- **Click "Retry":** The request might succeed on the second attempt
- **Try a simpler question:** Break your question into smaller parts

### Responses are slow
- **This is normal for complex questions:** Searching the book and generating answers can take 5-10 seconds
- **Look for the loading indicator:** A spinner with "Thinking..." means the assistant is working
- **If it takes >10 seconds:** You'll see a message "This is taking longer than usual..."

### Citations don't open the correct page
- **Check the URL:** Some citations might be relative paths (e.g., `/docs/kinematics/...`)
- **Try clicking the link again:** Browser navigation issues are rare but possible
- **Report broken citations:** If a citation consistently fails, let us know

### My conversation disappeared
- **Conversations are per-tab:** If you close the tab or open a new one, you start a new conversation
- **Conversations don't persist across sessions:** Closing your browser clears all conversations
- **This is intentional for privacy:** Your conversation history is not stored on the server

---

## Keyboard Shortcuts

- **Enter:** Send message
- **Shift+Enter:** Add new line in message (without sending)
- **Esc:** Close chat window (when chat is focused)
- **Tab:** Navigate through buttons and input fields

---

## Privacy and Data

### What data is stored?

**In your browser (sessionStorage):**
- Your current conversation (messages and session ID)
- Cleared when you close the tab

**On the server:**
- Session ID and conversation history (for 30 minutes after last activity)
- Used to maintain context for multi-turn conversations
- Automatically deleted after 30 minutes of inactivity

### What data is NOT stored?

- Your name or personal information
- Your IP address (not logged with conversations)
- Conversations across browser sessions
- Any data beyond 30 minutes of inactivity

---

## Technical Details

### Browser Compatibility

**Fully Supported:**
- Chrome 90+ (desktop and mobile)
- Firefox 88+ (desktop)
- Safari 14+ (desktop and iOS)
- Edge 90+ (desktop)

**Partially Supported:**
- Older browsers may have issues with text selection or sessionStorage

**Not Supported:**
- Internet Explorer

### Mobile Experience

- Chat window takes up full screen on mobile devices (<640px width)
- Tap the chat button to open/close
- Text selection works via long-press on mobile
- Keyboard automatically appears when typing

---

## FAQ

**Q: Can I download or export my conversation?**
A: Not yet. This feature may be added in the future.

**Q: Can I ask questions in languages other than English?**
A: The book content is in English, so the assistant works best with English questions. Other languages may work but are not officially supported.

**Q: How accurate are the answers?**
A: The assistant only uses information from the book, so answers are grounded in the book content. However, always verify by clicking on citations.

**Q: Can I ask the assistant to solve homework problems?**
A: The assistant can explain concepts from the book but cannot solve specific homework problems. It's designed to help you learn, not replace your own thinking.

**Q: What if I find an incorrect answer?**
A: Click on the citations to verify. If the answer is incorrect despite correct citations, please report it to the book maintainers.

**Q: Can I use this offline?**
A: No, the chat assistant requires an internet connection to work.

---

## Getting Help

If you encounter issues not covered in this guide:

1. **Check the browser console** (F12 → Console tab) for error messages
2. **Try a different browser** to see if the issue is browser-specific
3. **Report the issue** on the book's GitHub repository (include screenshots and error messages)

---

## What's Next?

After using the chat assistant, you might want to:

- **Explore cited sources** by clicking on citations
- **Ask deeper follow-up questions** to build understanding
- **Select specific passages** for targeted explanations
- **Compare your understanding** by asking the assistant to explain concepts you already know

Happy learning!
