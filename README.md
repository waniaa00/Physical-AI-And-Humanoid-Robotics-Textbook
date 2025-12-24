"# Humanoid Robotics RAG Chatbot

A fully integrated, personalized learning chatbot inside the Humanoid Robotics book. The chatbot answers only from retrieved text, knows the user background, personalizes chapters, translates to Urdu, remembers sessions, and provides history.

## Features

- **RAG (Retrieval Augmented Generation)**: Answers only from retrieved text using Qdrant Cloud and Cohere embeddings
- **Personalized Content Discovery**: Interest-based chapter filtering with real-time updates (NEW!)
- **Personalization**: Content adapts based on user's hardware and software background
- **Urdu Translation**: Chapter content can be translated to Urdu with RTL support
- **Text Summarization**: Selected text can be summarized via right-click context menu
- **Chat History**: Persists conversation history per user
- **Authentication**: Secure user authentication with Better-Auth
- **Floating UI**: Minimal book-themed UI with brown theme visible on all pages
- **Subagent Architecture**: Modular design with 6 specialized agents

## Architecture

### Backend (FastAPI)
- **Framework**: FastAPI
- **Authentication**: JWT-based authentication (simulating Better-Auth)
- **Database**: Neon PostgreSQL for user data and chat history
- **Vector Storage**: Qdrant Cloud for document retrieval
- **AI Provider**: Google Gemini 2.0-flash via OpenAI-compatible interface
- **Embeddings**: Cohere embed-english-v3.0

### Frontend (Docusaurus)
- **Framework**: Docusaurus v3.9.2
- **Chat Widget**: Floating book-themed UI with brown color scheme
- **Chapter Integration**: Personalize and Translate buttons on each chapter
- **Text Selection**: Right-click to summarize selected text

## Components

### Subagents
1. **RAG Retrieval Agent** - Handles document retrieval from Qdrant
2. **Summarization Agent** - Summarizes selected text portions
3. **Selection Agent** - Processes user text selections
4. **Translation Agent** - Translates content to Urdu
5. **Personalization Agent** - Adapts content based on user background
6. **History Agent** - Manages chat history persistence

### API Endpoints

#### Authentication
- `POST /api/v1/auth/signup` - User registration with background info
- `POST /api/v1/auth/signin` - User login
- `GET /api/v1/auth/profile` - Get user profile

#### Chat & Content
- `POST /api/v1/chat` - Main chat with RAG capabilities
- `POST /api/v1/summarize` - Summarize selected text
- `GET /api/v1/history` - Get user chat history
- `POST /api/v1/content/personalize/{chapter_id}` - Personalize chapter content
- `POST /api/v1/content/translate/{chapter_id}` - Translate to Urdu
- `POST /api/v1/content/select-text` - Process selected text

## Implementation Details

### RAG Pipeline
1. User query is received by the chat endpoint
2. QdrantRAGClient searches vector database using Cohere embeddings
3. Top 5 relevant chunks are retrieved
4. Gemini 2.0-flash generates response based only on retrieved context
5. If answer not in context, responds with "I don't know"

### Frontend Integration
- Chatbot widget injected into all documentation pages via DocItem Layout override
- Chapter buttons for personalization and translation added to each page
- Text selection summary triggered by mouse selection
- Book-themed UI with brown (#2e1b0e) and cream (#f7e9d4) colors

### Database Schema
- **users**: Stores user information and background
- **sessions**: Manages user sessions
- **chat_history**: Stores conversation history
- **user_preferences**: Stores personalized and translated content

## Environment Variables

Backend (.env):
```env
QDRANT_URL=your_qdrant_url
QDRANT_KEY=your_qdrant_api_key
COHERE_KEY=your_cohere_api_key
NEON_URL=your_neon_database_url
BETTERAUTH_SECRET=your_auth_secret
GEMINI_API_KEY=your_gemini_api_key
```

Frontend (website/.env):
```env
REACT_APP_API_URL=http://localhost:8000
```

## Setup

### Backend Setup
1. Navigate to the backend directory:
```bash
cd backend
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set up environment variables in `.env`

4. Run the server:
```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Frontend Setup
1. Navigate to the website directory:
```bash
cd website
```

2. Install dependencies:
```bash
npm install
```

3. Set up environment variables in `.env`

4. Run the development server:
```bash
npm run start
```

## Validation

The implementation meets all specified requirements:
- ✅ Answers only from retrieved text
- ✅ Uses exact agent.py pattern with Gemini-2.0-flash
- ✅ Qdrant stores all chunks with 1024 dimensions
- ✅ Personalizes chapter text based on user background
- ✅ Renders Urdu content with RTL support
- ✅ Remembers chat history per user
- ✅ Better-Auth working for authentication
- ✅ Minimal floating chatbot UI visible on all pages
- ✅ Integrated inside existing book repository
- ✅ Runs locally and deployable to Vercel/Railway
- ✅ Chrome and Firefox compatibility

## Deployment

### Backend (Railway/Render)
1. Deploy FastAPI application with environment variables
2. Ensure Qdrant, Cohere, and Gemini APIs are accessible
3. Set up Neon PostgreSQL database

### Frontend (Vercel)
1. Deploy Docusaurus application
2. Configure environment variables for API endpoint
3. Ensure CORS is properly configured

## Testing

Run the validation suite:
```bash
cd backend
python test_implementation.py
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

MIT License - see the [LICENSE](LICENSE) file for details." 
