from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
import google.generativeai as genai
import os
from dotenv import load_dotenv

# --- Load Environment Variables ---
load_dotenv()

# Database imports
try:
    from database import init_db, get_db_connection
except ImportError:
    def init_db(): pass
    def get_db_connection(): return None

# --- Configuration ---
# API Key check
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    print("CRITICAL WARNING: GEMINI_API_KEY is missing in .env")
else:
    genai.configure(api_key=GEMINI_API_KEY)

COLLECTION_NAME = "textbook_rag"

# --- Global Variables ---
embedding_model = None
qdrant_client = None
llm_model = None 

# --- Pydantic Models ---
class ChatMessage(BaseModel):
    message: str

# --- Lifespan Manager ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    global embedding_model, qdrant_client, llm_model
    
    # 1. Embedding Model
    print("Loading SentenceTransformer model...")
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
    
    # 2. Qdrant Client
    print("Initializing Qdrant client...")
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"), 
        api_key=os.getenv("QDRANT_API_KEY")
    )
    
    # 3. Gemini LLM (FIXED HERE)
    print("Initializing Gemini LLM...")
    try:
        # Using a model from your available list:
        llm_model = genai.GenerativeModel('gemini-flash-latest')
        print("Gemini Model Ready (Using gemini-2.0-flash).")
    except Exception as e:
        print(f"Failed to load Gemini: {e}")

    # 4. Database
    print("Initializing database...")
    try:
        init_db()
    except Exception as e:
        print(f"Database Init Error: {e}")

    yield

# --- FastAPI App ---
app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"status": "AI Backend Active"}

@app.post("/api/chat")
async def chat_handler(chat_message: ChatMessage):
    user_query = chat_message.message.strip()

    # 1. Greetings Check
    greetings = ["hi", "hello", "hey", "salam", "aoa", "hola"]
    if user_query.lower() in greetings:
        return {"response": "Hello! 👋 I am your Physical AI Textbook Assistant. Ask me anything about textbook"}

    if not embedding_model or not qdrant_client or not llm_model:
        raise HTTPException(status_code=503, detail="System is initializing or missing configuration.")

    try:
        # 2. Search Textbook (Retrieval)
        query_vector = embedding_model.encode(user_query).tolist()
        
        # Robust Search Logic
        search_results = []
        try:
            search_results = qdrant_client.search(
                collection_name=COLLECTION_NAME, query_vector=query_vector, limit=10
            )
        except AttributeError:
            result_obj = qdrant_client.query_points(
                collection_name=COLLECTION_NAME, query=query_vector, limit=10
            )
            search_results = result_obj.points

        # Combine Context
        context_parts = [res.payload['text'] for res in search_results if res.payload and 'text' in res.payload]
        context_text = "\n\n".join(context_parts)
        
        if not context_text:
            return {"response": "I couldn't find specific information in the textbook about that."}

        # 3. Generate Answer using Gemini (The AI Part)
        prompt = f"""
        You are a smart teaching assistant for a Robotics course.
        Answer the student's question based ONLY on the context provided below.
        
        Context:
        {context_text}
        
        Student Question: {user_query}
        
        Instructions:
        - Provide a concise, clear answer (max 3-4 sentences).
        - If the answer is technical, explain it simply.
        - Do not start with "Based on the context". Just answer.
        """

        response = llm_model.generate_content(prompt)
        final_answer = response.text

        # 4. Save to Postgres (Optional)
        try:
            conn = get_db_connection()
            if conn:
                cur = conn.cursor()
                cur.execute(
                    "INSERT INTO chat_history (user_query, ai_response) VALUES (%s, %s)",
                    (user_query, final_answer)
                )
                conn.commit()
                cur.close()
                conn.close()
        except Exception:
            pass 

        return {"response": final_answer}

    except Exception as e:
        print(f"Error: {e}")
        return {"response": "Sorry, I'm having trouble thinking right now. Please try again."}