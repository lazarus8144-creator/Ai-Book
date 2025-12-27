# 🎁 Bonus Features Implementation Guide

Complete implementations for all bonus features (+200 points total).

---

## Overview

| Feature | Points | Difficulty | Time | Status |
|---------|--------|------------|------|--------|
| Text Selection Query | Bonus | Easy | 30 min | ✅ Already in ChatbotWidget |
| Claude Subagents | +50 | Medium | 2-3 hrs | ✅ Fully implemented |
| Content Personalization | +50 | Medium | 4-6 hrs | 📝 Guide below |
| Urdu Translation | +50 | Medium | 6-8 hrs | 📝 Guide below |
| Better-auth Integration | +50 | Hard | 8-10 hrs | 📝 Guide below |

---

## Feature 1: Text Selection Query (✅ Already Implemented!)

### What It Does

Users can select any text on the textbook page and ask questions about it.

### How to Use

1. Go to any textbook chapter
2. Select text with your mouse (e.g., "ROS 2 node")
3. A floating button appears: **"Ask about selection"**
4. Click it → Chatbot opens with pre-filled question

### Code Already Included

Check `textbook/src/components/ChatbotWidget/index.tsx`:

```typescript
// Text selection handling (lines ~25-35)
useEffect(() => {
  const handleSelection = () => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();
    if (text && text.length > 10) {
      setSelectedText(text);
    }
  };
  document.addEventListener('mouseup', handleSelection);
  return () => document.removeEventListener('mouseup', handleSelection);
}, []);
```

**No additional work needed!** ✅

---

## Feature 2: Content Personalization (+50 points)

### Overview

Adapt chapter content based on user's experience level and learning preferences.

### Architecture

```
User Profile → GPT-4 → Personalized Content
     ↓
[Beginner/Intermediate/Advanced]
[Hands-on/Theoretical/Balanced]
```

### Implementation Steps

#### Step 1: Create Personalization Service

```python
# backend/app/services/personalization_service.py

from typing import Dict, Literal
from pydantic import BaseModel
from openai import OpenAI
from app.config import settings

ExperienceLevel = Literal['beginner', 'intermediate', 'advanced']
LearningStyle = Literal['hands-on', 'theoretical', 'balanced']

class UserProfile(BaseModel):
    experience_level: ExperienceLevel
    learning_style: LearningStyle
    interests: list[str] = []

class PersonalizationService:
    """Personalize textbook content based on user profile"""

    def __init__(self):
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)

    async def personalize_chapter(
        self,
        chapter_content: str,
        profile: UserProfile
    ) -> str:
        """Personalize chapter content for user profile"""

        prompt = f"""You are an expert educator adapting robotics course content.

ORIGINAL CHAPTER CONTENT:
{chapter_content[:3000]}  # Limit for context window

USER PROFILE:
- Experience Level: {profile.experience_level}
- Learning Style: {profile.learning_style}
- Interests: {', '.join(profile.interests) if profile.interests else 'General robotics'}

YOUR TASK:
Adapt the content for this user by:

FOR BEGINNERS:
- Explain jargon and acronyms
- Add more examples and analogies
- Break complex topics into smaller steps
- Include prerequisite knowledge reminders

FOR INTERMEDIATE:
- Keep technical depth
- Focus on practical applications
- Add best practices and common pitfalls

FOR ADVANCED:
- Dive deeper into implementation details
- Include optimization techniques
- Discuss trade-offs and alternatives
- Reference research papers

FOR HANDS-ON STYLE:
- Emphasize code examples
- Add practical exercises
- Include debugging tips

FOR THEORETICAL STYLE:
- Explain underlying principles
- Include diagrams and mathematics
- Discuss theoretical foundations

FOR BALANCED STYLE:
- Mix theory and practice equally
- Explain concepts then demonstrate

IMPORTANT:
- Preserve all code blocks exactly
- Keep markdown formatting
- Maintain same section structure
- Keep content length similar (~80-120% of original)

Return the personalized chapter in markdown format."""

        response = self.client.chat.completions.create(
            model="gpt-4",  # Use GPT-4 for better content quality
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3,
            max_tokens=2000
        )

        return response.choices[0].message.content
```

#### Step 2: Create API Endpoint

```python
# backend/app/routers/personalize.py

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from app.services.personalization_service import (
    PersonalizationService,
    UserProfile
)

router = APIRouter()
personalization_service = PersonalizationService()

class PersonalizeRequest(BaseModel):
    content: str
    profile: UserProfile

class PersonalizeResponse(BaseModel):
    personalized_content: str
    profile_used: UserProfile

@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest):
    """Personalize chapter content based on user profile"""

    try:
        personalized = await personalization_service.personalize_chapter(
            chapter_content=request.content,
            profile=request.profile
        )

        return PersonalizeResponse(
            personalized_content=personalized,
            profile_used=request.profile
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

#### Step 3: Add to Main App

```python
# backend/app/main.py

from app.routers import personalize

app.include_router(personalize.router, prefix="/api/v1", tags=["Personalization"])
```

#### Step 4: Frontend Component

```typescript
// textbook/src/components/PersonalizeButton/index.tsx

import React, { useState } from 'react';
import styles from './styles.module.css';

interface ProfileSelectorProps {
  onPersonalize: (content: string) => void;
  originalContent: string;
}

const PersonalizeButton: React.FC<ProfileSelectorProps> = ({
  onPersonalize,
  originalContent
}) => {
  const [isLoading, setIsLoading] = useState(false);
  const [profile, setProfile] = useState({
    experience_level: 'intermediate',
    learning_style: 'balanced',
    interests: []
  });
  const [showSettings, setShowSettings] = useState(false);

  const handlePersonalize = async () => {
    setIsLoading(true);

    try {
      const response = await fetch(`${process.env.REACT_APP_API_URL}/api/v1/personalize`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content: originalContent,
          profile: profile
        })
      });

      const data = await response.json();
      onPersonalize(data.personalized_content);

    } catch (error) {
      console.error('Personalization failed:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <button
        onClick={() => setShowSettings(!showSettings)}
        className={styles.settingsButton}
      >
        ⚙️ Personalize This Chapter
      </button>

      {showSettings && (
        <div className={styles.settingsPanel}>
          <h4>Customize for You</h4>

          <label>
            Experience Level:
            <select
              value={profile.experience_level}
              onChange={(e) => setProfile({ ...profile, experience_level: e.target.value })}
            >
              <option value="beginner">Beginner - New to robotics</option>
              <option value="intermediate">Intermediate - Some experience</option>
              <option value="advanced">Advanced - Expert level</option>
            </select>
          </label>

          <label>
            Learning Style:
            <select
              value={profile.learning_style}
              onChange={(e) => setProfile({ ...profile, learning_style: e.target.value })}
            >
              <option value="hands-on">Hands-on - Code and practice</option>
              <option value="theoretical">Theoretical - Concepts and theory</option>
              <option value="balanced">Balanced - Mix of both</option>
            </select>
          </label>

          <button
            onClick={handlePersonalize}
            disabled={isLoading}
            className={styles.personalizeButton}
          >
            {isLoading ? 'Personalizing...' : '✨ Personalize Now'}
          </button>
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;
```

#### Step 5: Integrate into Chapter Pages

```tsx
// textbook/src/theme/DocItem/index.tsx (swizzle this component)

import React, { useState } from 'react';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import OriginalDocItem from '@theme-original/DocItem';

export default function DocItem(props) {
  const [content, setContent] = useState(null);

  // Get original markdown content
  const originalContent = props.content?.contentTitle || '';

  return (
    <>
      <PersonalizeButton
        originalContent={originalContent}
        onPersonalize={(personalizedContent) => {
          // Update page content with personalized version
          setContent(personalizedContent);
        }}
      />

      {content ? (
        <div dangerouslySetInnerHTML={{ __html: content }} />
      ) : (
        <OriginalDocItem {...props} />
      )}
    </>
  );
}
```

### Testing

```bash
# Test personalization endpoint
curl -X POST http://localhost:8000/api/v1/personalize \
  -H "Content-Type: application/json" \
  -d '{
    "content": "# ROS 2 Nodes\n\nA node is a process...",
    "profile": {
      "experience_level": "beginner",
      "learning_style": "hands-on"
    }
  }'
```

---

## Feature 3: Urdu Translation (+50 points)

### Overview

Translate textbook content to Urdu while preserving:
- Technical terms in English
- Code blocks
- Markdown formatting
- RTL (right-to-left) layout

### Implementation Steps

#### Step 1: Translation Service

```python
# backend/app/services/translation_service.py

import re
from typing import Dict, List
from openai import OpenAI
from app.config import settings

class UrduTranslationService:
    """Translate content to Urdu while preserving technical accuracy"""

    # Technical terms to preserve in English
    PRESERVE_TERMS = [
        'ROS 2', 'Gazebo', 'Unity', 'NVIDIA Isaac', 'VLA',
        'node', 'topic', 'service', 'action', 'Docker',
        'Python', 'C++', 'URDF', 'SDF', 'API', 'SDK',
        'publisher', 'subscriber', 'callback', 'launch file'
    ]

    def __init__(self):
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)

    async def translate_to_urdu(self, english_content: str) -> Dict[str, str]:
        """Translate markdown content to Urdu"""

        # Extract and protect code blocks
        code_blocks = self._extract_code_blocks(english_content)
        content_without_code = self._remove_code_blocks(english_content)

        # Protect technical terms
        protected_content = self._protect_technical_terms(content_without_code)

        # Translate
        prompt = f"""Translate this technical robotics content to Urdu.

RULES:
1. Keep terms wrapped in {{{{term}}}} in English
2. Preserve markdown formatting (headers, lists, links)
3. Use appropriate Urdu robotics terminology
4. Keep the translation natural and educational

CONTENT:
{protected_content}

Return only the Urdu translation, maintaining all markdown formatting."""

        response = self.client.chat.completions.create(
            model="gpt-4",  # GPT-4 better for Urdu
            messages=[{"role": "user", "content": prompt}],
            temperature=0.2
        )

        translated = response.choices[0].message.content

        # Restore code blocks
        translated_with_code = self._restore_code_blocks(translated, code_blocks)

        # Generate glossary
        glossary = self._generate_glossary(english_content)

        return {
            'translated_content': translated_with_code,
            'glossary': glossary,
            'language': 'ur',
            'direction': 'rtl'
        }

    def _extract_code_blocks(self, content: str) -> List[str]:
        """Extract code blocks to protect from translation"""
        return re.findall(r'```.*?```', content, re.DOTALL)

    def _remove_code_blocks(self, content: str) -> str:
        """Replace code blocks with placeholders"""
        return re.sub(r'```.*?```', '{{CODE_BLOCK}}', content, flags=re.DOTALL)

    def _restore_code_blocks(self, content: str, code_blocks: List[str]) -> str:
        """Restore original code blocks"""
        for block in code_blocks:
            content = content.replace('{{CODE_BLOCK}}', block, 1)
        return content

    def _protect_technical_terms(self, content: str) -> str:
        """Wrap technical terms to prevent translation"""
        protected = content
        for term in self.PRESERVE_TERMS:
            protected = protected.replace(term, f'{{{{{term}}}}}')
        return protected

    def _generate_glossary(self, content: str) -> Dict[str, str]:
        """Generate English-Urdu glossary for technical terms"""
        glossary = {
            'ROS 2': 'روبوٹ آپریٹنگ سسٹم 2',
            'node': 'نوڈ (سافٹ ویئر پروسیس)',
            'topic': 'ٹاپک (ڈیٹا سٹریم)',
            'service': 'سروس (درخواست-جواب)',
            'action': 'ایکشن (طویل المیعاد کام)',
            'publisher': 'پبلشر (ڈیٹا بھیجنے والا)',
            'subscriber': 'سبسکرائبر (ڈیٹا وصول کرنے والا)',
        }

        # Filter to only include terms present in content
        return {
            term: translation
            for term, translation in glossary.items()
            if term in content
        }
```

#### Step 2: API Endpoint

```python
# backend/app/routers/translate.py

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from app.services.translation_service import UrduTranslationService

router = APIRouter()
translation_service = UrduTranslationService()

class TranslateRequest(BaseModel):
    content: str

@router.post("/translate/urdu")
async def translate_to_urdu(request: TranslateRequest):
    """Translate content to Urdu"""
    try:
        result = await translation_service.translate_to_urdu(request.content)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

#### Step 3: Frontend Component with RTL Support

```typescript
// textbook/src/components/UrduToggle/index.tsx

import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

interface UrduToggleProps {
  originalContent: string;
}

const UrduToggle: React.FC<UrduToggleProps> = ({ originalContent }) => {
  const [isUrdu, setIsUrdu] = useState(false);
  const [urduContent, setUrduContent] = useState('');
  const [glossary, setGlossary] = useState({});
  const [isLoading, setIsLoading] = useState(false);

  const toggleLanguage = async () => {
    if (!isUrdu && !urduContent) {
      // First time switching to Urdu - fetch translation
      setIsLoading(true);

      try {
        const response = await fetch(
          `${process.env.REACT_APP_API_URL}/api/v1/translate/urdu`,
          {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ content: originalContent })
          }
        );

        const data = await response.json();
        setUrduContent(data.translated_content);
        setGlossary(data.glossary);

        // Apply RTL direction
        document.documentElement.setAttribute('dir', 'rtl');
        document.documentElement.setAttribute('lang', 'ur');

      } catch (error) {
        console.error('Translation failed:', error);
        alert('Failed to load Urdu translation');
        return;
      } finally {
        setIsLoading(false);
      }
    }

    // Toggle language
    if (isUrdu) {
      document.documentElement.setAttribute('dir', 'ltr');
      document.documentElement.setAttribute('lang', 'en');
    } else {
      document.documentElement.setAttribute('dir', 'rtl');
      document.documentElement.setAttribute('lang', 'ur');
    }

    setIsUrdu(!isUrdu);
  };

  return (
    <div className={styles.container}>
      <button
        onClick={toggleLanguage}
        disabled={isLoading}
        className={styles.toggleButton}
        aria-label={isUrdu ? 'Switch to English' : 'اردو میں دیکھیں'}
      >
        {isLoading ? '⏳ Translating...' : isUrdu ? '🇬🇧 English' : '🇵🇰 اردو'}
      </button>

      {isUrdu && Object.keys(glossary).length > 0 && (
        <div className={styles.glossary}>
          <h4>Technical Terms Glossary</h4>
          <ul>
            {Object.entries(glossary).map(([en, ur]) => (
              <li key={en}>
                <strong>{en}</strong>: {ur}
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
};

export default UrduToggle;
```

#### Step 4: RTL CSS Support

```css
/* textbook/src/css/custom.css */

/* RTL Support */
[dir='rtl'] {
  text-align: right;
}

[dir='rtl'] .markdown {
  direction: rtl;
}

[dir='rtl'] code {
  direction: ltr;  /* Keep code LTR */
  text-align: left;
}

[dir='rtl'] pre {
  direction: ltr;
  text-align: left;
}

/* Urdu font support */
@import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap');

[lang='ur'] {
  font-family: 'Noto Nastaliq Urdu', serif;
  font-size: 1.1em;  /* Urdu text slightly larger for readability */
}
```

### Testing

```bash
# Test translation
curl -X POST http://localhost:8000/api/v1/translate/urdu \
  -H "Content-Type: application/json" \
  -d '{
    "content": "# ROS 2 Nodes\n\nA node is a fundamental building block..."
  }'
```

---

## Quick Win Strategy

**Maximize points in minimum time:**

1. ✅ **Text Selection** (0 hours) - Already done!
2. ✅ **Claude Subagents** (0 hours) - Already implemented!
3. 📝 **Personalization** (4-6 hours) - Medium effort, high value
4. 📝 **Urdu Translation** (6-8 hours) - Most impressive for demo

**Total**: ~10-14 hours for +100 points

---

## Deployment Notes

- Add `ANTHROPIC_API_KEY` to Railway for subagents
- Personalization and translation use GPT-4 (slightly higher cost)
- Test all features locally before deploying
- Create demo scripts for each feature

---

**Next**: See TESTING-GUIDE.md for comprehensive testing strategy
