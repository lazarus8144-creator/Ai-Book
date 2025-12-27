#!/usr/bin/env python3
"""
Document Ingestion Script for RAG Chatbot
==========================================

Processes all textbook markdown files and ingests them into Qdrant vector database.

Features:
- Semantic chunking with header-based boundaries
- Batch processing for efficient embeddings
- Progress tracking with detailed logging
- Incremental updates (re-ingestion safe)
- Error handling and retry logic

Usage:
    python scripts/ingest_docs.py --docs-path ../textbook/docs --verbose
    python scripts/ingest_docs.py --docs-path ../textbook/docs --batch-size 100
    python scripts/ingest_docs.py --single-file ../textbook/docs/module-1-ros2/01-introduction.md
"""

import argparse
import asyncio
import sys
import time
from pathlib import Path
from typing import List, Dict, Optional
import yaml
import re
import hashlib
from dataclasses import dataclass, asdict
from tqdm import tqdm
import tiktoken

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from sentence_transformers import SentenceTransformer


# ============================================================================
# Data Models
# ============================================================================

@dataclass
class ChunkMetadata:
    """Metadata for a document chunk"""
    module: str
    chapter: str
    title: str
    section: str
    url: str
    keywords: List[str]


@dataclass
class Chunk:
    """Document chunk with metadata"""
    text: str
    metadata: ChunkMetadata
    token_count: int
    chunk_id: str


# ============================================================================
# Document Processor
# ============================================================================

class DocumentProcessor:
    """Process markdown files into semantic chunks"""

    def __init__(self, max_tokens: int = 800, overlap: int = 100, base_url: str = "https://your-project.vercel.app"):
        self.max_tokens = max_tokens
        self.overlap = overlap
        self.base_url = base_url
        self.encoding = tiktoken.encoding_for_model("text-embedding-3-small")

    def process_file(self, file_path: Path) -> List[Chunk]:
        """Process a single markdown file into chunks"""
        try:
            content = file_path.read_text(encoding='utf-8')
        except Exception as e:
            print(f"⚠️  Error reading {file_path}: {e}")
            return []

        # Parse frontmatter
        frontmatter = self._parse_frontmatter(content)
        content_without_frontmatter = self._remove_frontmatter(content)

        # Extract metadata from file path
        metadata_base = self._extract_metadata(file_path, frontmatter)

        # Split by headers
        sections = self._split_by_headers(content_without_frontmatter)

        # Create chunks
        chunks = []
        for section_text, section_name in sections:
            section_chunks = self._chunk_section(section_text, section_name, metadata_base)
            chunks.extend(section_chunks)

        return chunks

    def _split_by_headers(self, content: str) -> List[tuple]:
        """Split content by markdown headers (##, ###) while preserving code blocks"""
        sections = []
        current_section = []
        current_header = "Introduction"

        lines = content.split('\n')
        in_code_block = False

        for line in lines:
            # Track code block state
            if line.strip().startswith('```'):
                in_code_block = not in_code_block

            # Detect headers (only outside code blocks)
            if not in_code_block and re.match(r'^#{2,3}\s+', line):
                if current_section:
                    sections.append(('\n'.join(current_section), current_header))
                    current_section = []
                current_header = line.lstrip('#').strip()

            current_section.append(line)

        # Add final section
        if current_section:
            sections.append(('\n'.join(current_section), current_header))

        return sections

    def _chunk_section(self, text: str, section_name: str, metadata_base: ChunkMetadata) -> List[Chunk]:
        """Chunk a section respecting token limits"""
        tokens = self.encoding.encode(text)

        # If section fits in one chunk, return it
        if len(tokens) <= self.max_tokens:
            return [self._create_chunk(text, section_name, metadata_base)]

        # Split into smaller chunks with overlap
        chunks = []
        start = 0

        while start < len(tokens):
            end = min(start + self.max_tokens, len(tokens))
            chunk_tokens = tokens[start:end]
            chunk_text = self.encoding.decode(chunk_tokens)

            chunks.append(self._create_chunk(chunk_text, section_name, metadata_base))

            # Move start with overlap
            start += (self.max_tokens - self.overlap)

        return chunks

    def _create_chunk(self, text: str, section: str, metadata_base: ChunkMetadata) -> Chunk:
        """Create a chunk with metadata"""
        metadata = ChunkMetadata(
            module=metadata_base.module,
            chapter=metadata_base.chapter,
            title=metadata_base.title,
            section=section,
            url=metadata_base.url,
            keywords=metadata_base.keywords
        )

        token_count = len(self.encoding.encode(text))
        chunk_id = self._generate_chunk_id(text, metadata)

        return Chunk(
            text=text,
            metadata=metadata,
            token_count=token_count,
            chunk_id=chunk_id
        )

    def _extract_metadata(self, file_path: Path, frontmatter: dict) -> ChunkMetadata:
        """Extract metadata from file path and frontmatter"""
        parts = file_path.parts

        # Extract module and chapter from path
        # e.g., textbook/docs/module-1-ros2/01-introduction.md
        module = "unknown"
        chapter = "unknown"

        for part in parts:
            if part.startswith('module-'):
                module = part
            if re.match(r'^\d+-.*\.md$', part):
                chapter = part.replace('.md', '')

        title = frontmatter.get('title', chapter.replace('-', ' ').title())
        keywords = frontmatter.get('keywords', [])

        # Build URL
        if module != "unknown" and chapter != "unknown":
            url = f"{self.base_url}/{module}/{chapter}"
        else:
            url = self.base_url

        return ChunkMetadata(
            module=module,
            chapter=chapter,
            title=title,
            section="",
            url=url,
            keywords=keywords if isinstance(keywords, list) else []
        )

    def _parse_frontmatter(self, content: str) -> dict:
        """Extract YAML frontmatter from markdown"""
        match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)
        if match:
            try:
                return yaml.safe_load(match.group(1)) or {}
            except:
                return {}
        return {}

    def _remove_frontmatter(self, content: str) -> str:
        """Remove YAML frontmatter from content"""
        return re.sub(r'^---\n.*?\n---\n', '', content, count=1, flags=re.DOTALL)

    def _generate_chunk_id(self, text: str, metadata: ChunkMetadata) -> str:
        """Generate deterministic chunk ID for deduplication"""
        content = f"{metadata.module}:{metadata.chapter}:{metadata.section}:{text[:100]}"
        return hashlib.md5(content.encode()).hexdigest()


# ============================================================================
# Embedding & Vector Store
# ============================================================================

class EmbeddingService:
    """Generate embeddings using sentence-transformers (FREE)"""

    def __init__(self):
        print(f"📥 Loading embedding model: {settings.EMBEDDING_MODEL}")
        self.model = SentenceTransformer(
            settings.EMBEDDING_MODEL,
            device=settings.EMBEDDING_DEVICE
        )
        self.dimension = self.model.get_sentence_embedding_dimension()
        print(f"✅ Embedding model loaded! Dimension: {self.dimension}")

    def get_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a batch of texts"""
        try:
            embeddings = self.model.encode(texts, convert_to_numpy=True, show_progress_bar=True)
            return [emb.tolist() for emb in embeddings]
        except Exception as e:
            print(f"⚠️  Embedding generation failed: {e}")
            raise


class VectorStoreService:
    """Manage Qdrant vector database"""

    def __init__(self, vector_size: int = 384):
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.vector_size = vector_size
        self._ensure_collection()

    def _ensure_collection(self):
        """Create collection if it doesn't exist"""
        collections = self.client.get_collections().collections
        collection_names = [c.name for c in collections]

        if self.collection_name not in collection_names:
            print(f"📦 Creating collection: {self.collection_name}")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,  # sentence-transformers dimension (384 for all-MiniLM-L6-v2)
                    distance=Distance.COSINE
                )
            )
            print(f"✅ Collection created successfully")
        else:
            print(f"✅ Collection '{self.collection_name}' already exists")

    def upsert_chunks(self, chunks: List[Chunk], embeddings: List[List[float]]):
        """Upsert chunks with embeddings to Qdrant"""
        points = []

        for chunk, embedding in zip(chunks, embeddings):
            # Convert chunk_id to numeric ID (first 8 hex chars as int)
            point_id = int(chunk.chunk_id[:8], 16)

            # Prepare payload
            payload = {
                "text": chunk.text,
                "module": chunk.metadata.module,
                "chapter": chunk.metadata.chapter,
                "title": chunk.metadata.title,
                "section": chunk.metadata.section,
                "url": chunk.metadata.url,
                "keywords": chunk.metadata.keywords,
                "token_count": chunk.token_count,
                "chunk_id": chunk.chunk_id
            }

            points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload=payload
            ))

        # Upsert to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def get_collection_stats(self) -> Dict:
        """Get collection statistics"""
        info = self.client.get_collection(self.collection_name)
        return {
            "points_count": info.points_count,
            "status": str(info.status)
        }


# ============================================================================
# Ingestion Pipeline
# ============================================================================

class IngestionPipeline:
    """Orchestrate document ingestion"""

    def __init__(self, base_url: str = "https://your-project.vercel.app", verbose: bool = False):
        self.processor = DocumentProcessor(base_url=base_url)
        self.embeddings = EmbeddingService()
        self.vector_store = VectorStoreService(vector_size=self.embeddings.dimension)
        self.verbose = verbose

    def find_markdown_files(self, docs_path: Path) -> List[Path]:
        """Find all markdown files to process"""
        files = []

        for md_file in docs_path.rglob("*.md"):
            # Skip tutorial files and certain patterns
            if any(skip in str(md_file) for skip in ['tutorial-', 'blog/', '.docusaurus/']):
                if self.verbose:
                    print(f"⏭️  Skipping: {md_file.name}")
                continue
            files.append(md_file)

        return sorted(files)

    async def ingest_documents(self, docs_path: Path, batch_size: int = 50):
        """Ingest all documents from a directory"""
        print(f"\n{'='*70}")
        print(f"📚 Document Ingestion Pipeline")
        print(f"{'='*70}\n")

        start_time = time.time()

        # Find files
        print(f"🔍 Searching for markdown files in: {docs_path}")
        files = self.find_markdown_files(docs_path)
        print(f"✅ Found {len(files)} files to process\n")

        if not files:
            print("❌ No files found. Check your --docs-path")
            return

        # Process files
        print(f"📄 Processing files into chunks...")
        all_chunks = []

        for md_file in tqdm(files, desc="Processing files", unit="file"):
            chunks = self.processor.process_file(md_file)
            all_chunks.extend(chunks)

            if self.verbose:
                print(f"  ✓ {md_file.name}: {len(chunks)} chunks")

        print(f"\n✅ Created {len(all_chunks)} total chunks")
        print(f"📊 Avg chunk size: {sum(c.token_count for c in all_chunks) / len(all_chunks):.0f} tokens\n")

        # Generate embeddings in batches
        print(f"🧠 Generating embeddings (batch size: {batch_size})...")
        all_embeddings = []

        for i in tqdm(range(0, len(all_chunks), batch_size), desc="Embedding batches", unit="batch"):
            batch = all_chunks[i:i+batch_size]
            texts = [chunk.text for chunk in batch]
            embeddings = self.embeddings.get_embeddings_batch(texts)
            all_embeddings.extend(embeddings)

            # Small delay to respect rate limits
            await asyncio.sleep(0.1)

        print(f"✅ Generated {len(all_embeddings)} embeddings\n")

        # Upload to Qdrant in batches
        print(f"☁️  Uploading to Qdrant...")

        for i in tqdm(range(0, len(all_chunks), batch_size), desc="Uploading batches", unit="batch"):
            batch_chunks = all_chunks[i:i+batch_size]
            batch_embeddings = all_embeddings[i:i+batch_size]
            self.vector_store.upsert_chunks(batch_chunks, batch_embeddings)

        # Final statistics
        elapsed = time.time() - start_time
        stats = self.vector_store.get_collection_stats()

        print(f"\n{'='*70}")
        print(f"✅ Ingestion Complete!")
        print(f"{'='*70}")
        print(f"📁 Files processed: {len(files)}")
        print(f"📦 Chunks created: {len(all_chunks)}")
        print(f"🎯 Vectors in Qdrant: {stats['points_count']}")
        print(f"⏱️  Time taken: {elapsed:.1f}s")
        print(f"{'='*70}\n")

    async def ingest_single_file(self, file_path: Path):
        """Ingest a single file (for updates)"""
        print(f"\n📄 Ingesting single file: {file_path.name}")

        chunks = self.processor.process_file(file_path)
        if not chunks:
            print(f"❌ No chunks created from {file_path.name}")
            return

        print(f"✅ Created {len(chunks)} chunks")

        # Generate embeddings
        texts = [chunk.text for chunk in chunks]
        embeddings = self.embeddings.get_embeddings_batch(texts)

        # Upload
        self.vector_store.upsert_chunks(chunks, embeddings)

        print(f"✅ Uploaded to Qdrant\n")


# ============================================================================
# CLI Interface
# ============================================================================

async def main():
    parser = argparse.ArgumentParser(
        description="Ingest textbook markdown files into Qdrant vector database",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Ingest all documents
  python scripts/ingest_docs.py --docs-path ../textbook/docs

  # Ingest with larger batches
  python scripts/ingest_docs.py --docs-path ../textbook/docs --batch-size 100

  # Update single file
  python scripts/ingest_docs.py --single-file ../textbook/docs/module-1-ros2/01-introduction.md

  # Verbose output
  python scripts/ingest_docs.py --docs-path ../textbook/docs --verbose
        """
    )

    parser.add_argument(
        '--docs-path',
        type=Path,
        help='Path to textbook docs directory (e.g., ../textbook/docs)'
    )

    parser.add_argument(
        '--single-file',
        type=Path,
        help='Process a single file instead of entire directory'
    )

    parser.add_argument(
        '--batch-size',
        type=int,
        default=50,
        help='Batch size for embeddings (default: 50)'
    )

    parser.add_argument(
        '--base-url',
        type=str,
        default='https://your-project.vercel.app',
        help='Base URL for source links (default: https://your-project.vercel.app)'
    )

    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose output'
    )

    args = parser.parse_args()

    # Validate arguments
    if not args.docs_path and not args.single_file:
        parser.error("Either --docs-path or --single-file must be provided")

    # Run ingestion
    pipeline = IngestionPipeline(base_url=args.base_url, verbose=args.verbose)

    try:
        if args.single_file:
            if not args.single_file.exists():
                print(f"❌ File not found: {args.single_file}")
                sys.exit(1)
            await pipeline.ingest_single_file(args.single_file)
        else:
            if not args.docs_path.exists():
                print(f"❌ Directory not found: {args.docs_path}")
                sys.exit(1)
            await pipeline.ingest_documents(args.docs_path, batch_size=args.batch_size)

        print("🎉 All done! Your RAG chatbot is ready to use.\n")

    except KeyboardInterrupt:
        print("\n\n⚠️  Ingestion interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Error during ingestion: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
