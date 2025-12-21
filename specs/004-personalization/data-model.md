# Data Model: Content Personalization

**Feature**: 004-personalization
**Date**: 2025-12-21
**Phase**: Phase 1 (Data Model)

This document defines the database schema, entity relationships, and data models for the personalization feature.

---

## Entity Relationship Diagram

```
┌──────────────────────────┐
│ User (Feature 003)       │
│ ─────────────────────    │
│ • id (UUID, PK)          │
│ • email                  │
│ • password_hash          │
└──────────┬───────────────┘
           │ 1
           │
           │ 1:1
           ▼
┌──────────────────────────┐
│ LearningProfile          │
│ (Feature 003)            │
│ ─────────────────────    │
│ • user_id (UUID, FK, PK) │
│ • skill_level            │◄────────────┐
│ • learning_goals         │             │
│ • prior_experience       │             │ Read-only
└──────────┬───────────────┘             │ (for personalization context)
           │ 1                            │
           │                              │
           │ 1:N                          │
           ▼                              │
┌──────────────────────────────────┐     │
│ PersonalizedContent (NEW)        │     │
│ ──────────────────────────────── │     │
│ • id (UUID, PK)                  │     │
│ • user_id (UUID, FK) ────────────┼─────┘
│ • chapter_id (String)            │
│ • original_content_hash (String) │
│ • personalized_markdown (Text)   │
│ • created_at (Timestamp)         │
│ • last_accessed_at (Timestamp)   │
│ • is_invalid (Boolean)           │
│ • invalidation_reason (String)   │
│ • invalidated_at (Timestamp)     │
└──────────────────────────────────┘

           ┌────────────────┐
           │ User           │
           └────┬───────────┘
                │ 1
                │
                │ 1:1
                ▼
┌──────────────────────────────────┐
│ PersonalizationPreferences (NEW) │
│ ──────────────────────────────── │
│ • user_id (UUID, FK, PK)         │
│ • verbosity_level (Enum)         │
│ • example_preference (Enum)      │
│ • code_comment_depth (Enum)      │
│ • created_at (Timestamp)         │
│ • updated_at (Timestamp)         │
└──────────────────────────────────┘

           ┌────────────────┐
           │ User           │
           └────┬───────────┘
                │ 1
                │
                │ 1:N
                ▼
┌───────────────────────────────────┐
│ ChapterRecommendation (NEW)       │
│ ───────────────────────────────── │
│ • user_id (UUID, FK)              │
│ • current_chapter_id (String, PK) │
│ • recommended_chapter_ids (JSON)  │
│ • reasoning (JSON)                │
│ • generated_at (Timestamp)        │
│ • expires_at (Timestamp)          │
└───────────────────────────────────┘

Composite Primary Keys:
- PersonalizedContent: (user_id, chapter_id, original_content_hash)
- ChapterRecommendation: (user_id, current_chapter_id)
```

---

## 1. PersonalizedContent Entity

### Purpose
Stores cached personalized versions of textbook chapters per user to avoid regeneration (SC-008: <1s retrieval).

### SQLAlchemy Model

```python
# rag-backend/app/personalization/models.py

from sqlalchemy import Column, String, Text, Boolean, DateTime, ForeignKey, Index
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from app.database.base import Base
import uuid
from datetime import datetime

class PersonalizedContent(Base):
    __tablename__ = "personalized_content"

    # Primary Key
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)

    # Foreign Keys
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)

    # Chapter Identification
    chapter_id = Column(String(100), nullable=False, index=True)
    # Examples: "1.1-ros2-intro", "2.3-gazebo-simulation", "3.4-isaac-navigation"

    # Content Storage
    personalized_markdown = Column(Text, nullable=False)
    # Full personalized chapter content in markdown format (average size: 50 KB)

    original_content_hash = Column(String(64), nullable=False)
    # SHA-256 hash of original chapter (first 16 chars: "sha256:abc123...")
    # Used to detect when original textbook chapter is updated

    # Cache Management
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    last_accessed_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    # Used for cache eviction (delete if not accessed in 90 days)

    is_invalid = Column(Boolean, nullable=False, default=False)
    # Set to True when user updates profile or preferences (triggers regeneration)

    invalidation_reason = Column(String(50), nullable=True)
    # "profile_update" | "preference_change" | "original_content_changed" | "manual"

    invalidated_at = Column(DateTime, nullable=True)

    # Relationships
    user = relationship("User", back_populates="personalized_content")

    # Composite Unique Constraint
    __table_args__ = (
        Index("idx_user_chapter", "user_id", "chapter_id", unique=True),
        Index("idx_last_accessed", "last_accessed_at"),  # For cache eviction queries
        Index("idx_invalid_content", "is_invalid"),  # For filtering valid cache
    )

    def __repr__(self):
        return f"<PersonalizedContent user_id={self.user_id} chapter={self.chapter_id} valid={not self.is_invalid}>"
```

### Pydantic Schemas

```python
# rag-backend/app/personalization/schemas.py

from pydantic import BaseModel, Field, UUID4
from datetime import datetime
from typing import Optional

class PersonalizedContentBase(BaseModel):
    chapter_id: str = Field(..., max_length=100, description="Chapter identifier (e.g., '1.2-ros2-nodes')")
    personalized_markdown: str = Field(..., description="Personalized chapter content in markdown")
    original_content_hash: str = Field(..., max_length=64, description="SHA-256 hash of original content")

class PersonalizedContentCreate(PersonalizedContentBase):
    user_id: UUID4

class PersonalizedContentResponse(PersonalizedContentBase):
    id: UUID4
    user_id: UUID4
    created_at: datetime
    last_accessed_at: datetime
    is_invalid: bool
    from_cache: bool = Field(default=True, description="Whether content was retrieved from cache")
    generation_time_ms: Optional[int] = Field(default=None, description="Time taken to generate (if not from cache)")

    class Config:
        from_attributes = True
```

### Database Indexes

1. **idx_user_chapter** (user_id, chapter_id): Unique composite index for fast cache lookups
2. **idx_last_accessed**: For cache eviction queries (find content not accessed in 90 days)
3. **idx_invalid_content**: Filter valid cache entries (is_invalid = false)

### Storage Estimation

- Average chapter: 2,000-5,000 words → ~50 KB personalized markdown
- 21 chapters × 3 skill levels = 63 baseline cached versions
- User growth: 100 users × 10 chapters personalized = 1,000 cached versions
- Storage: 1,000 × 50 KB = 50 MB (well within 0.5 GB Neon Postgres free tier)

---

## 2. PersonalizationPreferences Entity

### Purpose
Stores user-specific personalization settings (verbosity, example types, code comment depth) for fine-grained control (FR-020 to FR-023).

### SQLAlchemy Model

```python
# rag-backend/app/personalization/models.py

from sqlalchemy import Column, String, DateTime, ForeignKey, Enum as SQLEnum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from app.database.base import Base
from datetime import datetime
import enum

class VerbosityLevel(str, enum.Enum):
    CONCISE = "concise"
    MODERATE = "moderate"
    DETAILED = "detailed"

class ExamplePreference(str, enum.Enum):
    THEORETICAL = "theoretical"
    PRACTICAL = "practical"
    MIXED = "mixed"

class CodeCommentDepth(str, enum.Enum):
    MINIMAL = "minimal"
    STANDARD = "standard"
    EXTENSIVE = "extensive"

class PersonalizationPreferences(Base):
    __tablename__ = "personalization_preferences"

    # Primary Key (one-to-one with User)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), primary_key=True, index=True)

    # Preference Settings
    verbosity_level = Column(SQLEnum(VerbosityLevel), nullable=False, default=VerbosityLevel.MODERATE)
    # CONCISE: shorter explanations, fewer examples
    # MODERATE: balanced (default)
    # DETAILED: comprehensive explanations, more examples

    example_preference = Column(SQLEnum(ExamplePreference), nullable=False, default=ExamplePreference.MIXED)
    # THEORETICAL: academic examples, mathematical proofs
    # PRACTICAL: real-world projects, industry use cases
    # MIXED: balance of both

    code_comment_depth = Column(SQLEnum(CodeCommentDepth), nullable=False, default=CodeCommentDepth.STANDARD)
    # MINIMAL: only non-obvious logic
    # STANDARD: moderate commenting (default)
    # EXTENSIVE: line-by-line explanations

    # Timestamps
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationships
    user = relationship("User", back_populates="personalization_preferences")

    def __repr__(self):
        return f"<PersonalizationPreferences user_id={self.user_id} verbosity={self.verbosity_level}>"
```

### Pydantic Schemas

```python
# rag-backend/app/personalization/schemas.py

from pydantic import BaseModel, UUID4
from datetime import datetime
from app.personalization.models import VerbosityLevel, ExamplePreference, CodeCommentDepth

class PersonalizationPreferencesBase(BaseModel):
    verbosity_level: VerbosityLevel = VerbosityLevel.MODERATE
    example_preference: ExamplePreference = ExamplePreference.MIXED
    code_comment_depth: CodeCommentDepth = CodeCommentDepth.STANDARD

class PersonalizationPreferencesUpdate(BaseModel):
    verbosity_level: Optional[VerbosityLevel] = None
    example_preference: Optional[ExamplePreference] = None
    code_comment_depth: Optional[CodeCommentDepth] = None

class PersonalizationPreferencesResponse(PersonalizationPreferencesBase):
    user_id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

### Side Effect: Cache Invalidation

When preferences are updated (PUT /api/v1/personalization/preferences), all cached PersonalizedContent for the user must be invalidated:

```python
# rag-backend/app/personalization/router.py

@router.put("/preferences", response_model=PersonalizationPreferencesResponse)
async def update_preferences(
    preferences: PersonalizationPreferencesUpdate,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    # Update preferences
    db_prefs = db.query(PersonalizationPreferences).filter(
        PersonalizationPreferences.user_id == current_user.id
    ).first()

    if preferences.verbosity_level:
        db_prefs.verbosity_level = preferences.verbosity_level
    # ... update other fields

    db.commit()

    # Invalidate all cached personalized content (FR-022)
    invalidate_user_cache(db, current_user.id, reason="preference_change")

    return db_prefs
```

---

## 3. ChapterRecommendation Entity

### Purpose
Stores personalized "Next Chapter" recommendations based on user's learning goals and progress (FR-029 to FR-032).

### SQLAlchemy Model

```python
# rag-backend/app/personalization/models.py

from sqlalchemy import Column, String, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, JSON
from sqlalchemy.orm import relationship
from app.database.base import Base
from datetime import datetime, timedelta

class ChapterRecommendation(Base):
    __tablename__ = "chapter_recommendations"

    # Composite Primary Key
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), primary_key=True, index=True)
    current_chapter_id = Column(String(100), primary_key=True)

    # Recommendations (JSON array of chapter IDs)
    recommended_chapter_ids = Column(JSON, nullable=False)
    # Example: ["1.3-ros2-topics", "2.1-gazebo-intro", "1.4-ros2-services"]

    # Reasoning (JSON object with explanations)
    reasoning = Column(JSON, nullable=False)
    # Example:
    # {
    #   "1.3-ros2-topics": "Matches your goal: message passing systems",
    #   "2.1-gazebo-intro": "Next in learning path",
    #   "1.4-ros2-services": "Builds on current knowledge"
    # }

    # Cache Management (TTL: 7 days)
    generated_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    expires_at = Column(DateTime, nullable=False, default=lambda: datetime.utcnow() + timedelta(days=7))

    # Relationships
    user = relationship("User", back_populates="chapter_recommendations")

    def __repr__(self):
        return f"<ChapterRecommendation user_id={self.user_id} from={self.current_chapter_id} count={len(self.recommended_chapter_ids)}>"
```

### Pydantic Schemas

```python
# rag-backend/app/personalization/schemas.py

from pydantic import BaseModel, UUID4, Field
from datetime import datetime
from typing import Dict, List

class RecommendationItem(BaseModel):
    chapter_id: str = Field(..., description="Recommended chapter ID")
    title: str = Field(..., description="Chapter title")
    reason: str = Field(..., description="Why this chapter is recommended")

class ChapterRecommendationResponse(BaseModel):
    current_chapter_id: str
    recommendations: List[RecommendationItem] = Field(..., max_items=3, description="Top 3 recommended chapters")
    generated_at: datetime
    expires_at: datetime

    class Config:
        from_attributes = True
```

### TTL (Time-To-Live) Handling

Recommendations are regenerated every 7 days to reflect updated user progress:

```python
# rag-backend/app/personalization/service.py

def get_recommendations(db: Session, user_id: UUID, current_chapter_id: str) -> ChapterRecommendationResponse:
    """
    Get personalized chapter recommendations (with cache).
    """
    # Check cache
    cached = db.query(ChapterRecommendation).filter(
        ChapterRecommendation.user_id == user_id,
        ChapterRecommendation.current_chapter_id == current_chapter_id,
        ChapterRecommendation.expires_at > datetime.utcnow()  # Not expired
    ).first()

    if cached:
        return ChapterRecommendationResponse.from_orm(cached)

    # Generate new recommendations
    user_profile = get_user_profile(db, user_id)
    recommendations = recommend_next_chapters(current_chapter_id, user_profile, n=3)

    # Cache for 7 days
    db_recommendation = ChapterRecommendation(
        user_id=user_id,
        current_chapter_id=current_chapter_id,
        recommended_chapter_ids=[r["chapter_id"] for r in recommendations],
        reasoning={r["chapter_id"]: r["reason"] for r in recommendations},
    )
    db.add(db_recommendation)
    db.commit()

    return ChapterRecommendationResponse.from_orm(db_recommendation)
```

---

## Database Migration (Alembic)

### Migration Script

```python
# rag-backend/migrations/versions/002_personalization_tables.py

"""Add personalization tables

Revision ID: 002_personalization
Revises: 001_auth_tables
Create Date: 2025-12-21
"""

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers
revision = '002_personalization'
down_revision = '001_auth_tables'
branch_labels = None
depends_on = None

def upgrade():
    # PersonalizedContent table
    op.create_table(
        'personalized_content',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.id', ondelete='CASCADE'), nullable=False),
        sa.Column('chapter_id', sa.String(100), nullable=False),
        sa.Column('personalized_markdown', sa.Text, nullable=False),
        sa.Column('original_content_hash', sa.String(64), nullable=False),
        sa.Column('created_at', sa.DateTime, nullable=False),
        sa.Column('last_accessed_at', sa.DateTime, nullable=False),
        sa.Column('is_invalid', sa.Boolean, nullable=False, default=False),
        sa.Column('invalidation_reason', sa.String(50), nullable=True),
        sa.Column('invalidated_at', sa.DateTime, nullable=True),
    )
    op.create_index('idx_user_chapter', 'personalized_content', ['user_id', 'chapter_id'], unique=True)
    op.create_index('idx_last_accessed', 'personalized_content', ['last_accessed_at'])
    op.create_index('idx_invalid_content', 'personalized_content', ['is_invalid'])

    # PersonalizationPreferences table
    op.create_table(
        'personalization_preferences',
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.id', ondelete='CASCADE'), primary_key=True),
        sa.Column('verbosity_level', sa.Enum('concise', 'moderate', 'detailed', name='verbositylevel'), nullable=False),
        sa.Column('example_preference', sa.Enum('theoretical', 'practical', 'mixed', name='examplepreference'), nullable=False),
        sa.Column('code_comment_depth', sa.Enum('minimal', 'standard', 'extensive', name='codecommentdepth'), nullable=False),
        sa.Column('created_at', sa.DateTime, nullable=False),
        sa.Column('updated_at', sa.DateTime, nullable=False),
    )

    # ChapterRecommendation table
    op.create_table(
        'chapter_recommendations',
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.id', ondelete='CASCADE'), primary_key=True),
        sa.Column('current_chapter_id', sa.String(100), primary_key=True),
        sa.Column('recommended_chapter_ids', postgresql.JSON, nullable=False),
        sa.Column('reasoning', postgresql.JSON, nullable=False),
        sa.Column('generated_at', sa.DateTime, nullable=False),
        sa.Column('expires_at', sa.DateTime, nullable=False),
    )

def downgrade():
    op.drop_table('chapter_recommendations')
    op.drop_table('personalization_preferences')
    op.drop_table('personalized_content')
    op.execute('DROP TYPE verbositylevel')
    op.execute('DROP TYPE examplepreference')
    op.execute('DROP TYPE codecommentdepth')
```

---

## Storage Optimization Strategies

1. **Cache Eviction**: Delete PersonalizedContent not accessed in 90 days (cron job)
2. **Compression**: Store personalized_markdown as GZIP-compressed BYTEA (reduces size 60-70%)
3. **Deduplication**: Multiple users with same skill level get identical content → consider shared cache (future enhancement)

---

## Data Consistency Rules

1. **Cascade Delete**: When User is deleted, all PersonalizedContent, PersonalizationPreferences, and ChapterRecommendation records are deleted (ON DELETE CASCADE)
2. **Invalidation Trigger**: Profile update (skill_level, learning_goals, prior_experience changes) sets is_invalid=True on all PersonalizedContent
3. **TTL Enforcement**: ChapterRecommendation expires after 7 days, forcing regeneration with updated user progress

---

## Next Steps

1. Implement SQLAlchemy models in `rag-backend/app/personalization/models.py`
2. Create Alembic migration `002_personalization_tables.py`
3. Define Pydantic schemas in `rag-backend/app/personalization/schemas.py`
4. See [contracts/personalization-api.yaml](./contracts/personalization-api.yaml) for API endpoints using these models
