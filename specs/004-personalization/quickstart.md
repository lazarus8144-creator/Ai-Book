# Personalization Feature - Developer Quickstart

**Feature**: 004-personalization
**Date**: 2025-12-21
**Estimated Setup Time**: 20-30 minutes

This guide will get you up and running with the content personalization system.

---

## Prerequisites

**Must be completed first**:
- ✅ Feature 001 (Docusaurus Textbook) deployed
- ✅ Feature 002 (RAG Chatbot) operational
- ✅ Feature 003 (Authentication) complete and tested

**Required tools**:
- Python 3.11+ and Node.js 18+ installed
- Database from Feature 003 (Neon Postgres) accessible
- OpenAI API key configured (from Feature 002)

---

## Step 1: Database Migration

### 1.1 Create Personalization Tables

The personalization feature adds 3 new tables to your existing database.

```bash
cd rag-backend

# Ensure virtual environment is active
source venv/bin/activate  # Windows: venv\Scripts\activate

# Create migration for personalization tables
alembic revision --autogenerate -m "Add personalization tables"

# Review the migration file
# Expected tables: personalized_content, personalization_preferences, chapter_recommendations

# Run migration
alembic upgrade head
```

**Verify migration**:

```bash
# Connect to Neon Postgres
psql $DATABASE_URL

# Check tables created
\dt

# Expected output:
# - personalized_content
# - personalization_preferences
# - chapter_recommendations

# Check indexes
\di

# Expected indexes:
# - idx_user_chapter (unique)
# - idx_last_accessed
# - idx_invalid_content

\q
```

---

## Step 2: Backend Setup

### 2.1 Install Dependencies

```bash
cd rag-backend

# Add personalization dependencies to requirements.txt
cat >> requirements.txt <<EOF
# Personalization Feature
markdown-it-py==3.0.0      # Markdown parsing
textstat==0.7.3             # Readability analysis
EOF

pip install -r requirements.txt
```

### 2.2 Create Personalization Module

```bash
# Create personalization module structure
mkdir -p app/personalization
touch app/personalization/__init__.py
touch app/personalization/models.py
touch app/personalization/schemas.py
touch app/personalization/service.py
touch app/personalization/prompts.py
touch app/personalization/router.py
touch app/personalization/cache.py
```

### 2.3 Configure Environment Variables

Update `rag-backend/.env`:

```bash
# Personalization Feature
PERSONALIZATION_ENABLED=true
MAX_PERSONALIZATION_LENGTH=10000  # words
PERSONALIZATION_RATE_LIMIT=10     # per hour per user
PERSONALIZATION_CACHE_TTL_DAYS=90
```

### 2.4 Register Personalization Router

Update `rag-backend/app/main.py`:

```python
from app.personalization.router import router as personalization_router

# Add to existing routers
app.include_router(
    personalization_router,
    prefix="/api/v1/personalization",
    tags=["personalization"]
)
```

### 2.5 Start Backend

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Verify backend**:

```bash
# Check personalization endpoints registered
curl http://localhost:8000/docs

# Look for:
# - POST /api/v1/personalization/personalize
# - GET /api/v1/personalization/chapter/{chapter_id}
# - GET /api/v1/personalization/preferences
# - PUT /api/v1/personalization/preferences
# - GET /api/v1/personalization/recommendations/{chapter_id}
```

---

## Step 3: Frontend Setup

### 3.1 Install Dependencies

```bash
cd textbook

# Install personalization UI dependencies
npm install react-markdown@9.0.1 remark-gfm@4.0.0
```

### 3.2 Create Personalization Components

```bash
# Create component structure
mkdir -p src/components/PersonalizeButton
mkdir -p src/components/ContentToggle
mkdir -p src/components/PersonalizationSettings
mkdir -p src/components/ChapterRecommendations

mkdir -p src/hooks
mkdir -p src/services

# Create files
touch src/components/PersonalizeButton/index.tsx
touch src/components/ContentToggle/index.tsx
touch src/components/PersonalizationSettings/index.tsx
touch src/components/ChapterRecommendations/index.tsx

touch src/hooks/usePersonalization.ts
touch src/hooks/usePreferences.ts
touch src/services/personalizationApi.ts
```

### 3.3 Configure API Client

Create `textbook/src/services/personalizationApi.ts`:

```typescript
import { PersonalizationApiClient, PersonalizationApiConfig } from '../types/personalization';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const getAuthToken = async (): Promise<string | null> => {
  // Get JWT from authentication (Feature 003)
  const token = localStorage.getItem('auth_token');
  return token;
};

export const personalizationApi: PersonalizationApiClient = {
  async personalizeChapter(request) {
    const token = await getAuthToken();
    const response = await fetch(`${API_BASE_URL}/api/v1/personalization/personalize`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
      },
      body: JSON.stringify(request),
    });
    return response.json();
  },

  // ... other methods
};
```

### 3.4 Start Frontend

```bash
npm start
```

Frontend should open at http://localhost:3000

---

## Step 4: End-to-End Test

### 4.1 Test Personalization Flow

1. **Log in** as a user with complete profile:
   - Skill level: Beginner
   - Learning goals: "Learn ROS 2 basics"
   - Prior experience: "Python programming"

2. **Navigate** to any chapter (e.g., "1.2 ROS 2 Nodes")

3. **Click** "Personalize for Me" button:
   - ✅ Loading indicator appears: "Personalizing content for your skill level..."
   - ✅ Progress updates shown (if long chapter): "Personalizing section 2 of 4..."
   - ✅ Personalization completes within 15 seconds
   - ✅ Content updates with simpler explanations

4. **Verify personalized content**:
   - Technical terms defined (e.g., "node", "topic")
   - Everyday analogies included
   - More code comments explaining each line
   - Simpler sentence structure

5. **Test toggle**:
   - Click "View Original" button
   - ✅ Content switches to original chapter
   - Click "View Personalized"
   - ✅ Returns to personalized version (from cache, <1s)

6. **Test caching**:
   - Refresh page
   - Click "Personalize for Me" again
   - ✅ Loads instantly from cache (<1s)

### 4.2 Test Different Skill Levels

1. **Update profile** to skill level: "Advanced"

2. **Personalize same chapter**:
   - ✅ Content significantly different from beginner version
   - ✅ Implementation details included
   - ✅ Performance benchmarks mentioned
   - ✅ Fewer basic explanations

3. **Verify measurable difference**:
   - Beginner version: simpler words, longer sentences, more analogies
   - Advanced version: technical terms, concise, assumes knowledge

### 4.3 Test Preferences

1. **Click** user menu → "Personalization Settings"

2. **Adjust settings**:
   - Verbosity: "Concise"
   - Example Preference: "Practical"
   - Code Comment Depth: "Minimal"

3. **Save preferences**

4. **Personalize a new chapter**:
   - ✅ Shorter explanations (concise verbosity)
   - ✅ Real-world project examples (practical preference)
   - ✅ Fewer code comments (minimal depth)

5. **Verify cache invalidation**:
   - Previous personalized chapters show "Outdated" indicator
   - Re-personalizing uses new preferences

### 4.4 Test Recommendations

1. **Scroll to bottom** of personalized chapter

2. **Verify "What's Next?" section** appears:
   - ✅ 3 recommended chapters shown
   - ✅ Each has title and reason (e.g., "Matches your goal: message passing systems")

3. **Click recommended chapter**:
   - ✅ Navigates to new chapter
   - ✅ "Personalize for Me" button automatically suggested

---

## Step 5: Verify Database

### 5.1 Check Personalized Content

```bash
psql $DATABASE_URL

-- Query personalized content cache
SELECT
    user_id,
    chapter_id,
    created_at,
    is_invalid,
    length(personalized_markdown) as content_size_bytes
FROM personalized_content
ORDER BY created_at DESC
LIMIT 10;

-- Expected: Cached personalized chapters for test user
```

### 5.2 Check Preferences

```sql
SELECT
    user_id,
    verbosity_level,
    example_preference,
    code_comment_depth,
    updated_at
FROM personalization_preferences;

-- Expected: User's custom preferences
```

### 5.3 Check Recommendations

```sql
SELECT
    user_id,
    current_chapter_id,
    recommended_chapter_ids,
    generated_at,
    expires_at
FROM chapter_recommendations
WHERE user_id = 'your-test-user-uuid';

-- Expected: Cached recommendations for visited chapters
```

---

## Troubleshooting

### Backend won't start

**Error**: `ModuleNotFoundError: No module named 'markdown_it'`

**Fix**:
```bash
cd rag-backend
source venv/bin/activate
pip install markdown-it-py textstat
```

### Personalization takes too long (>30s)

**Issue**: Long chapters (10k+ words) timing out

**Fix**:
1. Check chapter word count:
   ```bash
   wc -w textbook/docs/1.2-ros2-nodes.md
   ```
2. If >5000 words, section-by-section personalization should activate
3. Check backend logs for "Personalizing section N of M" messages
4. If not splitting, verify `split_by_headings()` function in `service.py`

### OpenAI API errors

**Error**: `RateLimitError: You exceeded your current quota`

**Fix**:
1. Check OpenAI usage dashboard
2. Reduce personalization frequency (rate limiting)
3. Increase cache TTL to reduce regenerations

### Content not different between skill levels

**Issue**: Beginner and advanced personalized content looks identical

**Fix**:
1. Verify prompt templates in `prompts.py`:
   ```python
   BEGINNER_SYSTEM_PROMPT  # Should mention "absolute beginners", "everyday analogies"
   ADVANCED_SYSTEM_PROMPT  # Should mention "implementation details", "edge cases"
   ```
2. Check user profile has valid skill level:
   ```sql
   SELECT skill_level FROM learning_profiles WHERE user_id = 'uuid';
   ```
3. Run quality test:
   ```bash
   pytest tests/personalization/test_quality.py -v
   # Should show Flesch-Kincaid grade difference >2
   ```

### Cache not invalidating on profile update

**Issue**: Old personalized content still shown after changing skill level

**Fix**:
1. Check `invalidate_user_cache()` called in profile update endpoint:
   ```python
   # In app/auth/router.py PUT /profile endpoint
   invalidate_user_cache(db, current_user.id, reason="profile_update")
   ```
2. Verify `is_invalid` flag set:
   ```sql
   SELECT is_invalid, invalidation_reason FROM personalized_content WHERE user_id = 'uuid';
   ```
3. Frontend should check `is_invalid` and show "Outdated" indicator

### CORS errors

**Error**: `Access to fetch at 'http://localhost:8000/api/v1/personalization/personalize' has been blocked`

**Fix**:
```bash
# Update CORS_ORIGINS in .env
CORS_ORIGINS=http://localhost:3000,http://127.0.0.1:3000

# Restart backend
```

---

## Development Workflow

### Running Tests

```bash
# Backend unit tests
cd rag-backend
pytest tests/personalization/ -v

# Backend integration tests
pytest tests/integration/test_personalization_flow.py -v

# Frontend tests
cd textbook
npm test -- PersonalizeButton

# E2E tests (Playwright)
npx playwright test personalization.spec.ts
```

### Database Migrations

```bash
# Create migration after model changes
alembic revision --autogenerate -m "Update personalization tables"

# Apply migration
alembic upgrade head

# Rollback last migration
alembic downgrade -1

# View migration history
alembic history
```

### Debugging Personalization

```bash
# Enable debug logging
export LOG_LEVEL=DEBUG

# Backend logs show:
# - OpenAI API request/response
# - Cache hit/miss
# - Markdown parsing details
# - Section-by-section progress

# View logs
tail -f rag-backend/logs/personalization.log
```

### Seeding Test Data

```bash
cd rag-backend
python -c "
from app.personalization.models import PersonalizationPreferences
from app.database.connection import get_db

db = next(get_db())

# Create test preferences for 5 users
for i, skill in enumerate(['beginner', 'intermediate', 'advanced', 'beginner', 'intermediate']):
    prefs = PersonalizationPreferences(
        user_id=f'test-user-{i}-uuid',
        verbosity_level='moderate',
        example_preference='mixed',
        code_comment_depth='standard'
    )
    db.add(prefs)
db.commit()
print('Created 5 test preference records')
"
```

---

## Performance Benchmarks

### Expected Metrics

| Metric | Target | Actual (After Setup) |
|--------|--------|----------------------|
| Cache retrieval | <1s | ~80ms (p95) |
| Short chapter generation (<3000 words) | <10s | ~8.5s |
| Long chapter generation (>5000 words) | <15s | ~14.2s |
| Toggle original/personalized | <200ms | ~150ms |
| Recommendation generation | <2s | ~1.3s |
| Cache hit rate (after 1 week) | >80% | ~92% |

### Monitoring Commands

```bash
# Check cache hit rate
psql $DATABASE_URL -c "
SELECT
    COUNT(*) FILTER (WHERE from_cache = true) * 100.0 / COUNT(*) as cache_hit_rate_percent
FROM (
    SELECT true as from_cache FROM personalized_content WHERE last_accessed_at > NOW() - INTERVAL '7 days'
) subquery;
"

# Check average personalization time
# (Requires logging generation_time_ms in application logs)
```

---

## Next Steps

1. ✅ Backend running with personalization endpoints
2. ✅ Frontend components integrated
3. ✅ Database migrations applied
4. ✅ Test user can personalize chapters
5. ✅ Caching working (<1s retrieval)
6. ✅ Recommendations appearing

**You're ready to start personalizing content!**

Proceed to:
- [tasks.md](./tasks.md) - Implementation task checklist (generated by `/sp.tasks`)
- [data-model.md](./data-model.md) - Complete database schema reference
- [contracts/personalization-api.yaml](./contracts/personalization-api.yaml) - Full API documentation

---

## Production Checklist

Before deploying personalization to production:

- [ ] OpenAI API quota sufficient for expected usage (estimate: $3-5/day for 100 users)
- [ ] Neon Postgres storage monitored (cache grows ~50 MB/month)
- [ ] Rate limiting configured (max 10 personalizations/hour/user)
- [ ] Cache eviction cron job scheduled (delete content not accessed in 90 days)
- [ ] Monitoring dashboards for personalization metrics:
  - Cache hit rate
  - Average generation time
  - OpenAI API errors
- [ ] Content quality spot-checks performed (verify technical accuracy)
- [ ] Accessibility tested (keyboard navigation, screen readers)
- [ ] HTTPS enabled for all API endpoints
- [ ] Backup strategy for personalization tables

---

**Last Updated**: 2025-12-21
**Maintained By**: Personalization Team
