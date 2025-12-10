# Textbook Interface Contracts

## Translation Toggle Component API

### Component: TranslationToggle

**Purpose**: Allow users to switch between English and Urdu content

**Props**:
- `availableLanguages`: array of strings - List of available language codes (e.g., ['en', 'ur'])
- `currentLanguage`: string - Currently selected language code
- `onLanguageChange`: function - Callback when language selection changes

**Behavior**:
- Displays language selection button(s)
- When clicked, updates content to selected language
- Maintains current page context when switching languages

## Learning Outcome Component API

### Component: LearningOutcome

**Purpose**: Display learning outcomes for a chapter in a structured format

**Props**:
- `outcomes`: array of strings - List of learning outcomes
- `style`: string (optional) - Styling variant ('default', 'highlighted')

**Behavior**:
- Displays outcomes in a formatted list
- Can be styled differently based on context

## Exercise Component API

### Component: Exercise

**Purpose**: Display interactive exercises with instructions and expected outcomes

**Props**:
- `title`: string - Title of the exercise
- `difficulty`: string - Difficulty level ('beginner', 'intermediate', 'advanced')
- `type`: string - Type of exercise ('theoretical', 'practical', 'simulation', 'hardware')
- `instructions`: string - Step-by-step instructions
- `expectedOutcome`: string - What student should achieve

**Behavior**:
- Displays exercise in a formatted container
- May include interactive elements for solution submission

## Navigation API

### Sidebar Structure

**Format**: JSON structure defined in `sidebars.js`

**Properties**:
- `label`: string - Display name for the navigation item
- `type`: string - Type of item ('doc', 'category', 'link')
- `id`: string - Reference to the document
- `items`: array - Child navigation items (for categories)

## Content Format Contracts

### MDX Document Structure

**Required Frontmatter**:
- `title`: string - Document title
- `sidebar_position`: number - Position in sidebar navigation
- `description`: string - Brief description for SEO

**Optional Frontmatter**:
- `learning_outcomes`: array of strings - Learning objectives for the document
- `tags`: array of strings - Content tags for categorization

### Content Validation

**Rules**:
- All documents must be valid MDX
- All images must have alt text
- All links must be valid within the site context
- All code blocks must specify language for syntax highlighting