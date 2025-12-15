from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
import psycopg2
import os

from .database import init_db, get_db_connection

# --- Global Variables ---
embedding_model = None
qdrant_client = None
COLLECTION_NAME = "textbook_rag"

# --- Pydantic Models ---
class ChatMessage(BaseModel):
    message: str

# --- Lifespan Manager ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Load the model on startup
    global embedding_model
    global qdrant_client
    print("Loading SentenceTransformer model...")
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
    print("Model loaded.")
    
    print("Initializing Qdrant client...")
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"), 
        api_key=os.getenv("QDRANT_API_KEY")
    )
    print("Qdrant client initialized.")

    print("Initializing database...")
    init_db()
    print("Database initialized.")
    yield
    # Cleanup can be done here if needed

# --- FastAPI App ---
app = FastAPI(lifespan=lifespan)

# Configure CORS for Docusaurus (Port 3000)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- API Endpoints ---
@app.get("/")
def read_root():
    return {"status": "Backend Active", "module": "005-backend-rag-chatbot"}

@app.post("/api/chat")
async def chat_handler(chat_message: ChatMessage):
    if not embedding_model or not qdrant_client:
        raise HTTPException(status_code=503, detail="Model or Qdrant client not available")

    try:
        # 1. Encode the user's message
        query_vector = embedding_model.encode(chat_message.message).tolist()

        # 2. Search Qdrant for similar chunks
        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=3
        )

        # 3. Construct the response
        context = "\n\n---\n\n".join([result.payload['text'] for result in search_results])
        response_text = f"Based on the textbook, here is the relevant info:\n\n{context}"

        # 4. Save to Postgres
        try:
            conn = get_db_connection()
            cur = conn.cursor()
            cur.execute(
                "INSERT INTO chat_history (user_query, ai_response) VALUES (%s, %s)",
                (chat_message.message, response_text)
            )
            conn.commit()
            cur.close()
            conn.close()
        except Exception as e:
            print(f"Error saving to database: {e}")
            # Decide if you want to raise an exception or just log the error
            # For this case, we will just log it and continue

        return {"response": response_text}

    except Exception as e:
        print(f"An error occurred: {e}")
        raise HTTPException(status_code=500, detail="An error occurred during the chat process.")