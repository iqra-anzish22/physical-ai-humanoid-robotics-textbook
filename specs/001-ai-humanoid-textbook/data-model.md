# Data Model: Physical AI & Humanoid Robotics Textbook

## Entities

### Chapter
- **name**: string - The title of the chapter
- **slug**: string - URL-friendly identifier for the chapter
- **moduleOrder**: number - Position in the sequence of modules (1-6)
- **learningOutcomes**: array of strings - Learning objectives for the chapter
- **content**: string - The main content in MDX format
- **exercises**: array of Exercise objects - Practical exercises for the chapter
- **diagrams**: array of Diagram objects - Visual content for the chapter
- **references**: array of Reference objects - APA-formatted citations
- **metadata**: object - Additional properties like author, creation date, last updated

### Exercise
- **id**: string - Unique identifier for the exercise
- **title**: string - Brief title of the exercise
- **description**: string - Detailed description of what the exercise covers
- **difficulty**: string - Difficulty level (beginner, intermediate, advanced)
- **type**: string - Type of exercise (theoretical, practical, simulation, hardware)
- **instructions**: string - Step-by-step instructions for completing the exercise
- **expectedOutcome**: string - What the student should learn or achieve
- **chapterId**: string - Reference to the parent chapter

### Diagram
- **id**: string - Unique identifier for the diagram
- **title**: string - Title or caption for the diagram
- **description**: string - Description of what the diagram illustrates
- **filePath**: string - Path to the image file in the static directory
- **altText**: string - Alternative text for accessibility
- **chapterId**: string - Reference to the parent chapter
- **type**: string - Type of diagram (conceptual, architectural, workflow, etc.)

### Reference
- **id**: string - Unique identifier for the reference
- **apaCitation**: string - Full citation in APA format
- **title**: string - Title of the referenced work
- **authors**: array of strings - List of authors
- **year**: number - Publication year
- **source**: string - Where the reference was published (journal, book, website)
- **url**: string (optional) - Link to the source if available
- **chapterId**: string - Reference to the parent chapter

### TranslationSet
- **id**: string - Unique identifier for the translation set
- **languageCode**: string - Language code (e.g., 'ur' for Urdu, 'en' for English)
- **chapterId**: string - Reference to the parent chapter
- **content**: string - Translated content in MDX format
- **learningOutcomes**: array of strings - Translated learning outcomes
- **exerciseTranslations**: array of objects - Translated exercise content
- **diagramCaptions**: array of objects - Translated diagram captions
- **status**: string - Translation status (complete, partial, needs_review)

## Relationships

- Chapter 1--* Exercise: A chapter contains multiple exercises
- Chapter 1--* Diagram: A chapter contains multiple diagrams
- Chapter 1--* Reference: A chapter contains multiple references
- Chapter 1--* TranslationSet: A chapter can have multiple translations
- TranslationSet 1--* Exercise: A translation set contains translated exercises

## Validation Rules

- Chapter.moduleOrder must be between 1 and 6 (inclusive)
- Chapter.learningOutcomes must contain at least one learning outcome
- Chapter.content must be in valid MDX format
- Exercise.difficulty must be one of: 'beginner', 'intermediate', 'advanced'
- Reference.apaCitation must follow APA format standards
- TranslationSet.languageCode must be a valid ISO 639-1 or ISO 639-2 language code
- All required fields must be present for entity creation

## State Transitions

- Chapter: draft → review → published → archived
- Exercise: created → reviewed → approved → deprecated
- TranslationSet: not_started → in_progress → needs_review → complete