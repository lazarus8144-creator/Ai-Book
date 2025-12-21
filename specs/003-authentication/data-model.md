# Authentication Data Model

**Feature**: 003-authentication
**Date**: 2025-12-21
**Purpose**: Define database schema, SQLAlchemy models, and entity relationships

---

## Entity Relationship Diagram

```
┌─────────────────┐
│     users       │
│─────────────────│
│ id (PK)         │
│ email (UQ)      │
│ password_hash   │
│ is_active       │
│ created_at      │
│ updated_at      │
│ last_login_at   │
└─────────────────┘
        │
        │ 1:1
        ▼
┌─────────────────────┐
│ learning_profiles   │
│─────────────────────│
│ id (PK)             │
│ user_id (FK, UQ)    │
│ name                │
│ skill_level         │
│ learning_goals      │
│ prior_experience    │
│ created_at          │
│ updated_at          │
└─────────────────────┘

┌─────────────────┐
│     users       │
└─────────────────┘
        │
        │ 1:N
        ▼
┌─────────────────────┐
│     sessions        │
│─────────────────────│
│ id (PK)             │
│ user_id (FK)        │
│ token_hash          │
│ expires_at          │
│ device_info (JSON)  │
│ created_at          │
└─────────────────────┘

┌───────────────────────────┐
│ password_reset_tokens     │
│───────────────────────────│
│ id (PK)                   │
│ email (indexed)           │
│ token_hash                │
│ expires_at                │
│ used_at (nullable)        │
│ created_at                │
└───────────────────────────┘
```

---

## 1. User Entity

**Purpose**: Core account data for authentication

### Database Schema

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    is_active BOOLEAN DEFAULT TRUE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
    last_login_at TIMESTAMP WITH TIME ZONE
);

CREATE UNIQUE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_is_active ON users(is_active) WHERE is_active = TRUE;
```

### SQLAlchemy Model

```python
from sqlalchemy import Column, String, Boolean, DateTime, UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    is_active = Column(Boolean, default=True, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), nullable=False)
    last_login_at = Column(DateTime(timezone=True), nullable=True)

    # Relationships
    learning_profile = relationship("LearningProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")
    sessions = relationship("Session", back_populates="user", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email})>"
```

### Pydantic Schema (API)

```python
from pydantic import BaseModel, EmailStr, UUID4
from datetime import datetime
from typing import Optional

class UserBase(BaseModel):
    email: EmailStr

class UserCreate(UserBase):
    password: str  # Plain password (hashed before storing)

class UserResponse(UserBase):
    id: UUID4
    is_active: bool
    created_at: datetime
    last_login_at: Optional[datetime]

    class Config:
        from_attributes = True
```

### Field Constraints

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| id | UUID | PRIMARY KEY | Auto-generated |
| email | VARCHAR(255) | UNIQUE, NOT NULL | RFC 5322 validated |
| password_hash | VARCHAR(255) | NOT NULL | Bcrypt output (60 chars) |
| is_active | BOOLEAN | NOT NULL, DEFAULT TRUE | Soft delete support |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Immutable |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Auto-updated |
| last_login_at | TIMESTAMP | NULLABLE | Updated on login |

---

## 2. Learning Profile Entity

**Purpose**: Store user personalization data for content adaptation

### Database Schema

```sql
CREATE TABLE learning_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID UNIQUE NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    name VARCHAR(255) NOT NULL,
    skill_level VARCHAR(20) NOT NULL CHECK (skill_level IN ('beginner', 'intermediate', 'advanced')),
    learning_goals TEXT,
    prior_experience TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL
);

CREATE UNIQUE INDEX idx_learning_profiles_user_id ON learning_profiles(user_id);
CREATE INDEX idx_learning_profiles_skill_level ON learning_profiles(skill_level);
```

### SQLAlchemy Model

```python
from sqlalchemy import Column, String, Text, ForeignKey, DateTime, UUID, CheckConstraint
from sqlalchemy.orm import relationship
import uuid

class LearningProfile(Base):
    __tablename__ = "learning_profiles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), unique=True, nullable=False, index=True)
    name = Column(String(255), nullable=False)
    skill_level = Column(String(20), nullable=False)
    learning_goals = Column(Text, nullable=True)
    prior_experience = Column(Text, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), nullable=False)

    # Relationships
    user = relationship("User", back_populates="learning_profile")

    __table_args__ = (
        CheckConstraint("skill_level IN ('beginner', 'intermediate', 'advanced')", name="check_skill_level"),
    )

    def __repr__(self):
        return f"<LearningProfile(user_id={self.user_id}, skill_level={self.skill_level})>"
```

### Pydantic Schema (API)

```python
from enum import Enum

class SkillLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class LearningProfileBase(BaseModel):
    name: str
    skill_level: SkillLevel
    learning_goals: Optional[str] = None
    prior_experience: Optional[str] = None

class LearningProfileCreate(LearningProfileBase):
    pass

class LearningProfileUpdate(BaseModel):
    name: Optional[str] = None
    skill_level: Optional[SkillLevel] = None
    learning_goals: Optional[str] = None
    prior_experience: Optional[str] = None

class LearningProfileResponse(LearningProfileBase):
    id: UUID4
    user_id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

### Field Constraints

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| id | UUID | PRIMARY KEY | Auto-generated |
| user_id | UUID | FK → users.id, UNIQUE, NOT NULL | 1:1 relationship |
| name | VARCHAR(255) | NOT NULL | Display name |
| skill_level | VARCHAR(20) | NOT NULL, CHECK constraint | Enum validation |
| learning_goals | TEXT | NULLABLE | Free-form text |
| prior_experience | TEXT | NULLABLE | Free-form text |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Immutable |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Auto-updated |

---

## 3. Session Entity

**Purpose**: Track active user sessions for JWT validation and device auditing

### Database Schema

```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    device_info JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_token_hash ON sessions(token_hash);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
```

### SQLAlchemy Model

```python
from sqlalchemy import Column, ForeignKey, DateTime, UUID, String, JSON
from sqlalchemy.dialects.postgresql import JSONB
import uuid

class Session(Base):
    __tablename__ = "sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    token_hash = Column(String(255), nullable=False, index=True)
    expires_at = Column(DateTime(timezone=True), nullable=False, index=True)
    device_info = Column(JSONB, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)

    # Relationships
    user = relationship("User", back_populates="sessions")

    def __repr__(self):
        return f"<Session(user_id={self.user_id}, expires_at={self.expires_at})>"
```

### Pydantic Schema (API)

```python
from typing import Dict, Any

class DeviceInfo(BaseModel):
    user_agent: Optional[str] = None
    ip_address: Optional[str] = None
    platform: Optional[str] = None

class SessionCreate(BaseModel):
    user_id: UUID4
    token_hash: str
    expires_at: datetime
    device_info: Optional[DeviceInfo] = None

class SessionResponse(BaseModel):
    id: UUID4
    user_id: UUID4
    expires_at: datetime
    device_info: Optional[Dict[str, Any]]
    created_at: datetime

    class Config:
        from_attributes = True
```

### Field Constraints

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| id | UUID | PRIMARY KEY | Auto-generated |
| user_id | UUID | FK → users.id, NOT NULL | Multiple sessions per user |
| token_hash | VARCHAR(255) | NOT NULL, INDEXED | SHA256 hash of JWT |
| expires_at | TIMESTAMP | NOT NULL, INDEXED | TTL for cleanup |
| device_info | JSONB | NULLABLE | User agent, IP, platform |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Immutable |

### Cleanup Strategy

```python
# Background task to delete expired sessions (run hourly)
async def cleanup_expired_sessions():
    await session.execute(
        delete(Session).where(Session.expires_at < datetime.utcnow())
    )
    await session.commit()
```

---

## 4. Password Reset Token Entity

**Purpose**: Temporary tokens for password recovery flow

### Database Schema

```sql
CREATE TABLE password_reset_tokens (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) NOT NULL,
    token_hash VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE DEFAULT (NOW() + INTERVAL '1 hour') NOT NULL,
    used_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_password_reset_tokens_email ON password_reset_tokens(email);
CREATE INDEX idx_password_reset_tokens_token_hash ON password_reset_tokens(token_hash);
CREATE INDEX idx_password_reset_tokens_expires_at ON password_reset_tokens(expires_at);
```

### SQLAlchemy Model

```python
from sqlalchemy import Column, String, DateTime, UUID
from datetime import datetime, timedelta
import uuid

class PasswordResetToken(Base):
    __tablename__ = "password_reset_tokens"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), nullable=False, index=True)
    token_hash = Column(String(255), nullable=False, index=True)
    expires_at = Column(DateTime(timezone=True), nullable=False, default=lambda: datetime.utcnow() + timedelta(hours=1), index=True)
    used_at = Column(DateTime(timezone=True), nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)

    def is_valid(self) -> bool:
        """Check if token is valid (not expired and not used)"""
        return self.expires_at > datetime.utcnow() and self.used_at is None

    def mark_as_used(self):
        """Mark token as consumed"""
        self.used_at = datetime.utcnow()

    def __repr__(self):
        return f"<PasswordResetToken(email={self.email}, expires_at={self.expires_at})>"
```

### Pydantic Schema (API)

```python
class PasswordResetTokenCreate(BaseModel):
    email: EmailStr
    token_hash: str

class PasswordResetTokenResponse(BaseModel):
    id: UUID4
    email: EmailStr
    expires_at: datetime
    used_at: Optional[datetime]
    created_at: datetime

    class Config:
        from_attributes = True
```

### Field Constraints

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| id | UUID | PRIMARY KEY | Auto-generated |
| email | VARCHAR(255) | NOT NULL, INDEXED | User's email (no FK to support unregistered emails) |
| token_hash | VARCHAR(255) | NOT NULL, INDEXED | SHA256 hash of random token |
| expires_at | TIMESTAMP | NOT NULL, DEFAULT +1hr | TTL validation |
| used_at | TIMESTAMP | NULLABLE | Single-use enforcement |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Immutable |

### Cleanup Strategy

```python
# Background task to delete old tokens (run daily)
async def cleanup_old_reset_tokens():
    cutoff = datetime.utcnow() - timedelta(days=7)
    await session.execute(
        delete(PasswordResetToken).where(PasswordResetToken.created_at < cutoff)
    )
    await session.commit()
```

---

## Database Migrations

### Alembic Migration (Initial)

```python
"""Create authentication tables

Revision ID: 001_auth_tables
Revises:
Create Date: 2025-12-21
"""

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

revision = '001_auth_tables'
down_revision = None
branch_labels = None
depends_on = None

def upgrade():
    # Create users table
    op.create_table(
        'users',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('email', sa.String(255), unique=True, nullable=False),
        sa.Column('password_hash', sa.String(255), nullable=False),
        sa.Column('is_active', sa.Boolean(), nullable=False, server_default='true'),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('last_login_at', sa.DateTime(timezone=True), nullable=True),
    )
    op.create_index('idx_users_email', 'users', ['email'], unique=True)

    # Create learning_profiles table
    op.create_table(
        'learning_profiles',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('name', sa.String(255), nullable=False),
        sa.Column('skill_level', sa.String(20), nullable=False),
        sa.Column('learning_goals', sa.Text(), nullable=True),
        sa.Column('prior_experience', sa.Text(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE'),
        sa.CheckConstraint("skill_level IN ('beginner', 'intermediate', 'advanced')", name='check_skill_level'),
    )
    op.create_index('idx_learning_profiles_user_id', 'learning_profiles', ['user_id'], unique=True)

    # Create sessions table
    op.create_table(
        'sessions',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('token_hash', sa.String(255), nullable=False),
        sa.Column('expires_at', sa.DateTime(timezone=True), nullable=False),
        sa.Column('device_info', postgresql.JSONB(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE'),
    )
    op.create_index('idx_sessions_user_id', 'sessions', ['user_id'])
    op.create_index('idx_sessions_token_hash', 'sessions', ['token_hash'])
    op.create_index('idx_sessions_expires_at', 'sessions', ['expires_at'])

    # Create password_reset_tokens table
    op.create_table(
        'password_reset_tokens',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('email', sa.String(255), nullable=False),
        sa.Column('token_hash', sa.String(255), nullable=False),
        sa.Column('expires_at', sa.DateTime(timezone=True), nullable=False),
        sa.Column('used_at', sa.DateTime(timezone=True), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
    )
    op.create_index('idx_password_reset_tokens_email', 'password_reset_tokens', ['email'])
    op.create_index('idx_password_reset_tokens_token_hash', 'password_reset_tokens', ['token_hash'])
    op.create_index('idx_password_reset_tokens_expires_at', 'password_reset_tokens', ['expires_at'])

def downgrade():
    op.drop_table('password_reset_tokens')
    op.drop_table('sessions')
    op.drop_table('learning_profiles')
    op.drop_table('users')
```

---

## Data Validation Rules

### Email Validation
- RFC 5322 compliant (via Pydantic's `EmailStr`)
- Max length: 255 characters
- Case-insensitive storage (lowercase before insert)

### Password Requirements
- Min length: 8 characters (enforced in API, not DB)
- Max length: 128 characters (bcrypt limitation)
- No character restrictions (allow special chars, unicode)
- Stored as bcrypt hash only (never plaintext)

### Skill Level
- Enum: `beginner`, `intermediate`, `advanced`
- Database CHECK constraint enforces valid values
- Pydantic enum for type safety

### Learning Goals / Prior Experience
- Max length: No limit (TEXT type)
- Optional fields (can be empty)
- Sanitized for XSS before storage

---

## Indexing Strategy

### Performance Indexes
- `users.email`: Fast login queries (UNIQUE index)
- `sessions.token_hash`: Fast session validation
- `password_reset_tokens.token_hash`: Fast reset validation

### Cleanup Indexes
- `sessions.expires_at`: Efficient expired session cleanup
- `password_reset_tokens.expires_at`: Efficient token cleanup

### Foreign Key Indexes
- `learning_profiles.user_id`: Join optimization (already UNIQUE)
- `sessions.user_id`: User session lookup

---

## Next Steps

1. ✅ Database schema defined
2. ✅ SQLAlchemy models created
3. ✅ Pydantic schemas for API
4. ⏳ API contracts (OpenAPI spec)
5. ⏳ Quickstart guide
6. ⏳ Alembic migration implementation
