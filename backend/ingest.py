import os
import glob
import uuid
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
import google.generativeai as genai

# --- 1. Load Environment Variables ---
current_dir = os.path.dirname(os.path.abspath(__file__))
dotenv_path = os.path.join(current_dir, ".env")
load_dotenv(dotenv_path)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Docs Folder Path
DOCS_DIR = os.path.abspath(os.path.join(current_dir, "..", "physical-ai-book", "docs"))
COLLECTION_NAME = "textbook_rag"

# --- 2. Initial Checks ---
if not GEMINI_API_KEY:
    print(f"❌ ERROR: GEMINI_API_KEY missing in {dotenv_path}")
else:
    print(f"✅ Key Found: {GEMINI_API_KEY[:5]}*** (Length: {len(GEMINI_API_KEY)})")

# --- 3. Initialize Qdrant ---
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

def setup_qdrant_collection():
    print(f"🗑️ Recreating collection '{COLLECTION_NAME}' (Size: 768)...")
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
    )
    print("✅ Collection created successfully.")

def process_documents():
    # ✅ FORCE CONFIGURE HERE
    genai.configure(api_key=GEMINI_API_KEY)
    
    search_pattern_md = os.path.join(DOCS_DIR, "**", "*.md")
    search_pattern_mdx = os.path.join(DOCS_DIR, "**", "*.mdx")
    doc_files = glob.glob(search_pattern_md, recursive=True) + glob.glob(search_pattern_mdx, recursive=True)
    
    print(f"📁 Found {len(doc_files)} documents to process.")

    all_chunks = []
    for file_path in doc_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                for i in range(0, len(content), 1000):
                    chunk = content[i:i+1000]
                    if chunk.strip():
                        all_chunks.append({"text": chunk, "source": os.path.basename(file_path)})
        except Exception as e:
            print(f"❌ Error reading {file_path}: {e}")

    points = []
    print("🧠 Generating Google Embeddings (text-embedding-004)...")
    
    for i, chunk_data in enumerate(all_chunks):
        try:
            # ✅ Directly calling the API
            result = genai.embed_content(
                model="models/text-embedding-004",
                content=chunk_data["text"],
                task_type="retrieval_document"
            )
            
            points.append(
                models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=result['embedding'],
                    payload=chunk_data
                )
            )
            
            if (i + 1) % 5 == 0:
                print(f"   Processed {i + 1} chunks...")

        except Exception as e:
            print(f"❌ Error at chunk {i}: {e}")

    if points:
        print(f"📤 Uploading {len(points)} points to Qdrant...")
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
            wait=True 
        )
        print("🚀 Upload complete!")

if __name__ == "__main__":
    print("--- STARTING INGESTION ---")
    try:
        setup_qdrant_collection()
        process_documents()
        print("✅ ALL DONE! Your database is ready.")
    except Exception as e:
        print(f"❌ Fatal Error: {e}")