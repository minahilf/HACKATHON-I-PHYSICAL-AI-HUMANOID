import os
import glob
import uuid
from dotenv import load_dotenv
from openai import OpenAI
from qdrant_client import QdrantClient, models

# Load environment variables from .env file
load_dotenv()

# --- Configuration ---
DOCS_DIR = "../physical-ai-book/docs"
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
COLLECTION_NAME = "textbook_rag"

# --- Initialize Clients ---
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
openai_client = OpenAI(api_key=OPENAI_API_KEY)

def setup_qdrant_collection():
    """Checks if the Qdrant collection exists and creates it if it doesn't."""
    try:
        qdrant_client.get_collection(collection_name=COLLECTION_NAME)
        print(f"Collection '{COLLECTION_NAME}' already exists.")
    except Exception:
        print(f"Collection '{COLLECTION_NAME}' not found. Creating it now...")
        qdrant_client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
        )
        print("Collection created successfully.")

def process_documents():
    """
    Finds all Markdown files, chunks them, generates embeddings,
    and uploads them to a Qdrant collection.
    """
    # Find all .md and .mdx files recursively
    doc_files = glob.glob(os.path.join(DOCS_DIR, "**", "*.md*"), recursive=True)
    
    all_chunks = []
    for file_path in doc_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            # Simple chunking by character
            for i in range(0, len(content), 1000):
                chunk = content[i:i+1000]
                all_chunks.append({"text": chunk, "source": os.path.basename(file_path)})

    # Generate embeddings for all chunks
    points = []
    for chunk_data in all_chunks:
        response = openai_client.embeddings.create(
            input=chunk_data["text"],
            model="text-embedding-3-small"
        )
        embedding = response.data[0].embedding
        
        points.append(
            models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload=chunk_data
            )
        )

    # Upload points to Qdrant
    if points:
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
            wait=True  # Wait for the operation to complete
        )

if __name__ == "__main__":
    print("Starting ingestion process...")
    setup_qdrant_collection()
    process_documents()
    print("Ingestion Complete!")
