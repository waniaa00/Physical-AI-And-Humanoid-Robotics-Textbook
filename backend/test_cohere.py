"""
Quick diagnostic script to test Cohere API key
Run this to verify your Cohere key is working
"""
import os
from dotenv import load_dotenv
import cohere

load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")

if not COHERE_API_KEY:
    print("[ERROR] COHERE_API_KEY not found in environment")
    exit(1)

print(f"[OK] COHERE_KEY found: {COHERE_API_KEY[:10]}...")

try:
    co = cohere.Client(COHERE_API_KEY)

    # Test embedding generation
    print("\n[TEST] Testing embedding generation...")
    response = co.embed(
        texts=["test query"],
        model='embed-english-v3.0',
        input_type='search_query'
    )

    print(f"[SUCCESS] Embedding generated successfully!")
    print(f"   Dimension: {len(response.embeddings[0])}")
    print(f"   First 5 values: {response.embeddings[0][:5]}")

except Exception as e:
    print(f"\n[ERROR] Cohere API Error: {str(e)}")
    print("\nPossible causes:")
    print("1. API key is invalid or expired")
    print("2. Trial key has reached its limit")
    print("3. Rate limit exceeded")
    print("4. Network/connection issue")
    print("\nSolution: Get a new API key from https://dashboard.cohere.com/api-keys")
