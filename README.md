# Physical AI & Humanoid Robotics

An interactive textbook platform for learning Physical AI and Humanoid Robotics, featuring RAG-powered AI tutoring, personalized learning paths, and Urdu translation support.

## 🚀 Features

- **📚 Comprehensive Course Material**: Master ROS 2, NVIDIA Isaac, and Humanoid Robotics.
- **🤖 RAG-Powered AI Tutor**: Get instant answers based on course content.
- **🌍 Urdu Translation**: Technical content translated with Urdu support.
- **⚡ Personalized Learning**: Content adjusts to your skill level.
- **🔐 Authentication**: Secure user authentication system
- **📱 Responsive Design**: Built with Docusaurus for optimal viewing on all devices

## 🛠️ Quick Start

### 1. Clone the Repository
```bash
git clone https://github.com/Rao-Faizan/physical-ai-robotics.git
cd physical-ai-robotics
```

## 🛠️ Tech Stack

### Frontend
- **Docusaurus 3.9.2**: Modern static site generator
- **React 19**: UI framework
- **TypeScript**: Type-safe development
- **Better Auth**: Authentication system

### Backend
- **FastAPI**: High-performance Python web framework
- **OpenAI GPT-4**: AI-powered responses
- **Qdrant**: Vector database for RAG system
- **SQLite**: Local database for user data

## 📦 Installation

### Prerequisites
- Node.js 20+
- Python 3.9+
- Docker & Docker Compose (optional)

### Setup

1. **Clone the repository**
```bash
git clone https://github.com/Rao-Faizan/physical-ai-robotics.git
cd physical-ai-robotics
```

2. **Backend Setup**
```bash
cd backend
pip install -r requirements.txt

# Copy environment variables
cp .env.example .env
# Edit .env and add your API keys
```

3. **Frontend Setup**
```bash
cd frontend
npm install
```

4. **Configure Environment Variables**

Edit `backend/.env` with your credentials:
- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: Your Qdrant cloud URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `SECRET_KEY`: A secure secret key (min 32 characters)

## 🚀 Running the Application

### Using Docker Compose (Recommended)

```bash
docker-compose up
```

Access the application:
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- API Docs: http://localhost:8000/docs

### Manual Setup

**Terminal 1 - Backend:**
```bash
cd backend
python main.py
```

**Terminal 2 - Frontend:**
```bash
cd frontend
npm start
```

## 📖 Usage

1. **Browse Course Content**: Navigate through comprehensive modules on Physical AI
2. **Use AI Tutor**: Ask questions and get personalized explanations
3. **Translate to Urdu**: Switch language for Urdu content
4. **Personalize Learning**: Set your preferences for customized content
5. **Track Progress**: Sign up to save your learning progress

## 🌐 Deployment

### GitHub Pages

The project is configured for GitHub Pages deployment:

```bash
cd frontend
npm run build
npm run deploy
```

Your site will be available at: `https://rao-faizan.github.io/physical-ai-robotics/`

## 📁 Project Structure

```
physical-ai-robotics/
├── backend/                 # FastAPI backend
│   ├── api/                # API routes
│   ├── app/                # Core application logic
│   ├── rag/                # RAG system implementation
│   ├── main.py             # Application entry point
│   └── .env.example        # Environment variables template
├── frontend/               # Docusaurus frontend
│   ├── docs/               # Course documentation
│   ├── src/                # React components
│   ├── static/             # Static assets
│   └── docusaurus.config.ts # Docusaurus configuration
├── docker-compose.yml      # Docker orchestration
└── README.md              # This file
```

## 🔑 API Endpoints

- `GET /`: API information
- `GET /health`: Health check
- `POST /api/chat/query`: Chat with AI tutor
- `POST /api/auth/signup`: User registration
- `POST /api/auth/signin`: User login
- `POST /api/translate`: Translate content to Urdu
- `GET /api/personalize`: Get personalized content

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is open source and available under the MIT License.

## 👨‍💻 Author

**Faizan Raza**
- GitHub: [@Rao-Faizan](https://github.com/Rao-Faizan)

## 🙏 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- Powered by [OpenAI](https://openai.com/)
- Vector search by [Qdrant](https://qdrant.tech/)

---

**Note**: Remember to add your own API keys in the `.env` file before running the application.
