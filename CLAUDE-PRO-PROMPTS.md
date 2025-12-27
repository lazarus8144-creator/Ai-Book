# Claude Pro Prompts for Hackathon Completion

These are specialized prompts you can use with **Claude Pro** to complete your Physical AI Textbook hackathon project.

---

## 🔍 PROMPT 1: Complete Status Check & Deployment

```
I'm working on a hackathon project: Physical AI & Humanoid Robotics Textbook with RAG Chatbot.

Please perform a comprehensive audit:

1. SCAN THE REPOSITORY:
   - Check all files in /textbook/docs/ (textbook content)
   - Check all files in /backend/app/ (RAG chatbot backend)
   - Check deployment configurations
   - Check documentation completeness

2. IDENTIFY WHAT'S DONE VS MISSING:
   - Textbook: 4 modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA)
   - RAG Chatbot: FastAPI, Qdrant, embeddings, LLM services
   - Claude Subagents: SubagentOrchestrator implementation
   - Deployment: Railway, Vercel configs
   - Bonuses: Better-auth, Personalization, Urdu translation

3. CREATE ACTIONABLE TASKS:
   - List what needs immediate deployment
   - List what can be added for bonus points
   - Estimate time for each task
   - Prioritize by ROI (points per hour)

4. CHECK THESE FILES:
   - HACKATHON-STATUS-REPORT.md (current status)
   - .specify/memory/constitution.md (project requirements)
   - DEPLOY-TO-RAILWAY.txt (deployment credentials)
   - docs/BONUS-FEATURES.md (bonus feature code)

5. PROVIDE:
   - Completion percentage (out of 300 points)
   - Critical blockers preventing deployment
   - Recommended next steps
   - Estimated time to 150 points (base)
   - Estimated time to 200+ points (with bonuses)

Be specific about file paths, exact steps, and realistic time estimates.
```

---

## 🚀 PROMPT 2: Deploy to Production

```
I need to deploy my Physical AI Textbook project to production.

CURRENT STATE:
- Git repository initialized locally
- GitHub username: lazarus8144-creator
- Groq API key: (in backend/.env)
- Qdrant credentials: (in backend/.env)
- All code ready in: /home/kali/Downloads/Ai-Book-main/

DEPLOYMENT CREDENTIALS:
- File: DEPLOY-TO-RAILWAY.txt (has all environment variables)
- File: RAILWAY-CREDENTIALS.md (deployment reference)

PLEASE HELP ME:

1. CREATE GITHUB REPOSITORY:
   - Name: ai-textbook-chatbot
   - Public repository
   - Guide me through creation
   - Then push the code

2. DEPLOY BACKEND TO RAILWAY:
   - Read DEPLOY-TO-RAILWAY.txt for credentials
   - Guide me step-by-step through Railway setup
   - Root directory: backend
   - Environment variables: from DEPLOY-TO-RAILWAY.txt
   - Verify health check endpoint

3. DEPLOY FRONTEND TO VERCEL:
   - Root directory: textbook
   - Environment variable: REACT_APP_API_URL
   - Connect to Railway backend
   - Verify build succeeds

4. UPDATE CORS:
   - Update Railway environment variables
   - Add Vercel URL to ALLOWED_ORIGINS
   - Test CORS is working

5. END-TO-END TESTING:
   - Test chatbot on production site
   - Verify answers + source citations
   - Check dark mode
   - Test mobile responsive
   - Verify performance (<3s RAG queries)

Provide exact commands, URLs, and step-by-step instructions.
```

---

## 🎨 PROMPT 3: Improve Book Quality & UI

```
I have a Docusaurus textbook for Physical AI & Humanoid Robotics with 4 modules and 30+ chapters.

Location: /home/kali/Downloads/Ai-Book-main/textbook/

PLEASE ENHANCE:

1. NAVIGATION IMPROVEMENTS:
   - Add breadcrumb navigation
   - Add progress indicators per module
   - Add "Previous/Next Chapter" buttons
   - Improve sidebar organization
   - Add module completion tracking

2. CONTENT QUALITY:
   - Review all chapters for:
     - Technical accuracy
     - Code example completeness
     - Learning objective clarity
     - Consistent formatting
   - Add missing code examples
   - Improve explanations where needed
   - Add practical exercises

3. VISUAL ENHANCEMENTS:
   - Add Mermaid diagrams for:
     - ROS 2 architecture
     - Gazebo simulation pipeline
     - NVIDIA Isaac workflow
     - VLA system overview
   - Add architecture diagrams
   - Optimize existing images
   - Add hardware component images

4. CODE IMPROVEMENTS:
   - Ensure all code blocks have:
     - Syntax highlighting
     - Copy buttons
     - Language labels
     - Comments
   - Test that code examples are runnable
   - Add "Try it yourself" sections

5. ACCESSIBILITY:
   - Check WCAG AA compliance
   - Add alt text to images
   - Verify color contrast (4.5:1)
   - Test keyboard navigation
   - Add ARIA labels where needed

6. PERFORMANCE:
   - Optimize image sizes
   - Enable lazy loading
   - Minimize CSS/JS
   - Check Lighthouse scores
   - Target: 90+ performance, 90+ accessibility

7. SEARCH ENHANCEMENTS:
   - Improve search relevance
   - Add search filters by module
   - Add search suggestions
   - Highlight search terms in results

Provide specific file paths, code changes, and before/after examples.
```

---

## 🌐 PROMPT 4: Add Urdu Translation (+50 Bonus Points)

```
I want to add Urdu translation feature to my Docusaurus textbook for +50 bonus points.

REQUIREMENT:
- Logged users can translate chapters to Urdu by pressing a button
- RTL (right-to-left) layout support
- Technical terms preserved in English
- Code blocks remain in English

I HAVE COMPLETE CODE IN:
- File: docs/BONUS-FEATURES.md (Section 4: Urdu Translation)

PLEASE IMPLEMENT:

1. TRANSLATION SERVICE (Backend):
   - File: backend/app/services/translation_service.py
   - Use Google Translate API
   - Preserve code blocks (don't translate)
   - Preserve technical terms
   - Cache translations
   - Handle RTL text properly

2. DOCUSAURUS RTL SUPPORT:
   - File: textbook/docusaurus.config.ts
   - Add i18n configuration:
     - locales: ['en', 'ur']
     - RTL direction for Urdu
   - Configure language switcher

3. LANGUAGE SWITCHER COMPONENT:
   - File: textbook/src/components/LanguageSwitcher.tsx
   - Button: "اردو / English"
   - Toggle between English and Urdu
   - Preserve reading position
   - Show loading state during translation

4. FRONTEND INTEGRATION:
   - Add language switcher to navbar
   - Add "Translate to Urdu" button at chapter start
   - Show original on re-click
   - Persist language preference

5. TESTING:
   - Translate Module 1, Chapter 1
   - Verify RTL layout works
   - Verify code blocks remain LTR
   - Verify technical terms preserved
   - Test navigation in both languages

TECHNICAL STACK:
- Google Translate API (or alternative)
- Docusaurus i18n
- React state management
- Translation caching

Use the code from docs/BONUS-FEATURES.md as a starting point.
Provide complete implementation with all file changes.
```

---

## 🎯 PROMPT 5: Add Content Personalization (+50 Bonus Points)

```
I want to add content personalization feature for +50 bonus points.

REQUIREMENT:
- Logged users can personalize chapters by pressing a button
- Personalization based on:
  - Experience level: beginner/intermediate/advanced
  - Learning style: hands-on/theoretical/balanced
  - Background: robotics, AI, programming experience

I HAVE COMPLETE CODE IN:
- File: docs/BONUS-FEATURES.md (Section 3: Content Personalization)

PLEASE IMPLEMENT:

1. DATABASE SETUP (Neon Postgres):
   - Sign up at: https://neon.tech
   - Create database: hackathon_textbook
   - Create table: user_profiles
     - Columns: user_id, experience_level, learning_style, domain_familiarity
   - Connection string in .env

2. PERSONALIZATION SERVICE (Backend):
   - File: backend/app/services/personalization_service.py
   - Use OpenAI GPT-4o-mini
   - Adapt chapter content based on user profile:
     - Beginner: More examples, simpler explanations
     - Advanced: Edge cases, optimizations, deeper concepts
     - Hands-on: More code, less theory
     - Theoretical: More background, fewer examples
   - Cache personalized content

3. USER PROFILE STORAGE:
   - File: backend/app/services/user_service.py
   - CRUD operations for user profiles
   - Store in Neon Postgres
   - Session-based (no auth required initially)

4. FRONTEND UI:
   - File: textbook/src/components/PersonalizationPrompt.tsx
   - Show at chapter start:
     - "Personalize this chapter for you"
     - Experience level dropdown
     - Learning style dropdown
     - Domain familiarity checkboxes
     - "Apply" button
   - Replace chapter content with personalized version
   - Show "Revert to original" option

5. API ENDPOINTS:
   - POST /api/v1/personalize
     - Input: chapter_md, user_profile
     - Output: personalized_md
   - GET /api/v1/profile/:session_id
   - PUT /api/v1/profile/:session_id

6. TESTING:
   - Create 3 test profiles:
     - Beginner + Hands-on
     - Intermediate + Balanced
     - Advanced + Theoretical
   - Personalize Module 1, Chapter 1
   - Verify content differs significantly
   - Verify code examples preserved
   - Performance: <3s personalization

Use code from docs/BONUS-FEATURES.md as starting point.
Include complete implementation with database schema.
```

---

## 🔐 PROMPT 6: Add Better-auth Signup/Signin (+50 Bonus Points)

```
I want to implement user authentication using Better-auth for +50 bonus points.

REQUIREMENT:
- Signup/Signin using https://www.better-auth.com/
- At signup, ask background questions:
  - Software background (none/beginner/intermediate/expert)
  - Hardware background (none/hobbyist/professional/researcher)
  - Programming languages known
  - Robotics experience
  - AI/ML experience
- Store user profiles in Neon Postgres

PLEASE IMPLEMENT:

1. BETTER-AUTH SETUP:
   - Install: npm install better-auth
   - Configure OAuth providers:
     - Google OAuth
     - GitHub OAuth
   - Environment variables:
     - BETTER_AUTH_SECRET
     - GOOGLE_CLIENT_ID
     - GOOGLE_CLIENT_SECRET
     - GITHUB_CLIENT_ID
     - GITHUB_CLIENT_SECRET

2. BACKEND AUTH INTEGRATION:
   - File: backend/app/services/auth_service.py
   - Validate JWT tokens from better-auth
   - Session management
   - User profile CRUD

3. DATABASE SCHEMA (Neon Postgres):
   ```sql
   CREATE TABLE users (
     id UUID PRIMARY KEY,
     email TEXT UNIQUE,
     name TEXT,
     created_at TIMESTAMP DEFAULT NOW()
   );

   CREATE TABLE user_profiles (
     user_id UUID REFERENCES users(id),
     software_background TEXT,
     hardware_background TEXT,
     programming_languages JSONB,
     robotics_experience TEXT,
     ai_ml_experience TEXT,
     created_at TIMESTAMP DEFAULT NOW()
   );
   ```

4. SIGNUP FLOW:
   - Component: textbook/src/components/SignupFlow.tsx
   - Step 1: Choose OAuth provider
   - Step 2: Background questionnaire:
     - "What's your software background?"
     - "What's your hardware background?"
     - "Which programming languages do you know?"
     - "Do you have robotics experience?"
     - "Do you have AI/ML experience?"
   - Step 3: Complete signup
   - Redirect to personalized homepage

5. PROTECTED FEATURES:
   - Personalization (requires auth)
   - Translation (requires auth)
   - Chat history saving
   - Progress tracking

6. SESSION MANAGEMENT:
   - Store JWT in httpOnly cookie
   - Refresh token rotation
   - Session expiry: 7 days
   - Auto-logout on expiry

7. FRONTEND INTEGRATION:
   - Login button in navbar
   - User profile dropdown
   - Logout functionality
   - Protected route wrapper

Provide complete implementation with all OAuth setup steps.
```

---

## 🧪 PROMPT 7: Complete Testing & Quality Assurance

```
I need comprehensive testing for my hackathon project before submission.

PROJECT:
- Physical AI & Humanoid Robotics Textbook
- RAG Chatbot with Claude Subagents
- Location: /home/kali/Downloads/Ai-Book-main/

PLEASE CREATE AND RUN:

1. BACKEND TESTS:
   - Unit tests:
     - Test RAG pipeline
     - Test embedding service
     - Test LLM service
     - Test vector store
     - Test subagent orchestrator
   - Integration tests:
     - Test /api/v1/query endpoint
     - Test /api/v1/health endpoint
     - Test CORS configuration
     - Test rate limiting
   - Coverage target: >70%

2. FRONTEND TESTS:
   - Playwright E2E tests:
     - Navigation through all modules
     - Search functionality
     - Chatbot widget interaction
     - Dark mode toggle
     - Mobile responsive
     - Accessibility (WCAG AA)
   - Component tests:
     - ChatbotWidget
     - LearningObjectives
     - HardwareRequirements
     - ProgressIndicator

3. PERFORMANCE TESTS:
   - RAG query response time (<3s)
   - Page load time (<2s)
   - Vector search latency (<500ms)
   - Concurrent users (100+)
   - Lighthouse scores:
     - Performance: >90
     - Accessibility: >90
     - Best Practices: >90
     - SEO: >90

4. SECURITY AUDIT:
   - API key exposure check
   - CORS configuration
   - SQL injection prevention
   - XSS prevention
   - Rate limiting effectiveness
   - Input validation

5. ACCESSIBILITY AUDIT:
   - WCAG 2.1 AA compliance
   - Color contrast (4.5:1)
   - Keyboard navigation
   - Screen reader compatibility
   - Focus indicators
   - Alt text on images

6. MANUAL TESTING CHECKLIST:
   - Ask chatbot 10 test questions
   - Verify source citations link correctly
   - Test on multiple browsers (Chrome, Firefox, Safari)
   - Test on multiple devices (desktop, tablet, mobile)
   - Test dark mode throughout site
   - Verify all code examples render correctly

7. LOAD TESTING:
   - Simulate 100 concurrent users
   - Test chatbot under load
   - Verify no degradation
   - Check error rates
   - Monitor response times

Provide test scripts, expected results, and pass/fail criteria.
```

---

## 📊 PROMPT 8: Create Demo Presentation

```
I need to create a compelling demo presentation for hackathon judges.

PROJECT:
- Physical AI & Humanoid Robotics Textbook
- RAG Chatbot with Claude Subagents
- Points earned: [YOUR SCORE]/300

PLEASE CREATE:

1. PRESENTATION SLIDES (10 slides max):
   - Slide 1: Title + Project Overview
   - Slide 2: Problem Statement (Why this textbook?)
   - Slide 3: Solution Architecture
   - Slide 4: Textbook Content (4 modules)
   - Slide 5: RAG Chatbot Demo
   - Slide 6: Claude Subagents (+50 bonus)
   - Slide 7: Bonus Features Implemented
   - Slide 8: Technical Stack & Free Tier
   - Slide 9: Impact & Learning Outcomes
   - Slide 10: Future Enhancements

2. DEMO SCRIPT (5 minutes):
   - 0:00-0:30: Introduction + problem
   - 0:30-1:30: Textbook navigation demo
   - 1:30-3:00: RAG chatbot demo:
     - Ask: "What is ROS 2?"
     - Show answer + sources
     - Click source link
     - Ask: "How do I debug ROS nodes?"
     - Show code example insertion
   - 3:00-3:30: Claude Subagents demo:
     - Show accuracy scoring
     - Show follow-up questions
   - 3:30-4:30: Bonus features (if implemented)
   - 4:30-5:00: Impact + Q&A

3. DEMO VIDEO (3 minutes):
   - Screen recording walkthrough
   - Voiceover script
   - Highlight key features:
     - Comprehensive content
     - Working RAG chatbot
     - Source citations
     - Dark mode
     - Mobile responsive
     - Free tier architecture

4. README.md UPDATE:
   - Project banner/logo
   - Badges (build status, license, etc.)
   - Features list
   - Screenshots
   - Quick start guide
   - Tech stack diagram
   - Live demo links:
     - Textbook: https://...vercel.app
     - API: https://...railway.app
   - Scoring breakdown
   - Team/author info

5. HIGHLIGHT ACHIEVEMENTS:
   - Points earned: X/300
   - Features implemented
   - Free tier architecture ($0/month)
   - Comprehensive documentation
   - Claude Code + Spec-Kit Plus usage
   - Test coverage
   - Performance metrics

Provide presentation outline, demo script, and README content.
```

---

## 🎯 QUICK REFERENCE

### Which Prompt to Use When:

| Your Goal | Use Prompt | Time |
|-----------|------------|------|
| Check what's done/missing | #1 | 5 min |
| Deploy to production | #2 | 30 min |
| Improve textbook quality | #3 | 4-6 hrs |
| Add Urdu translation | #4 | 6-8 hrs |
| Add personalization | #5 | 4-6 hrs |
| Add authentication | #6 | 8-10 hrs |
| Complete testing | #7 | 3-4 hrs |
| Prepare presentation | #8 | 2-3 hrs |

### Recommended Order:

1. ✅ Prompt #1 (Status check)
2. ✅ Prompt #2 (Deploy) → **150 points**
3. ⚠️ Prompt #4 (Urdu) → **200 points**
4. ⚠️ Prompt #3 (Quality)
5. ⚠️ Prompt #7 (Testing)
6. ✅ Prompt #8 (Demo)

---

## 💡 PRO TIPS

1. **Copy-paste these prompts into Claude Pro exactly as written**
2. **Provide context**: Tell Claude Pro you're working on a hackathon
3. **Point to files**: Reference HACKATHON-STATUS-REPORT.md
4. **Ask follow-up questions**: Claude Pro can clarify and adapt
5. **Save responses**: Keep Claude Pro's outputs for reference

---

**File**: CLAUDE-PRO-PROMPTS.md
**Created**: 2025-12-27
**Purpose**: Complete hackathon project efficiently
**Target Score**: 200-250 points
