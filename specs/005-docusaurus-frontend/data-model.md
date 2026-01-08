# Data Model: Docusaurus-based Frontend for Physical AI & Humanoid Robotics Book

## Overview
This document defines the key data structures and entities for the Docusaurus-based frontend of the Physical AI & Humanoid Robotics book. It outlines the relationships between different content types and system components that will be managed through the Docusaurus site structure.

## Core Entities

### 1. Homepage
- **Description**: Main landing page with book title, description, and entry point to the documentation
- **Attributes**:
  - id: String (unique identifier: "homepage")
  - title: String ("Physical AI & Humanoid Robotics")
  - description: String ("Comprehensive guide to Physical AI & Humanoid Robotics - From foundational concepts to advanced robotic nervous systems")
  - tagline: String ("Building Intelligent Robots with NVIDIA Isaac Ecosystem")
  - featuredModules: Array[Module] (featured modules on homepage)
  - navigationLinks: Array[NavigationLink] (top-level navigation options)
  - readBookEntryPoint: EntryPoint (primary "Read Book" button/link)
- **Relationships**:
  - Links to multiple Module entities
  - Contains entry points to documentation sections

### 2. Module
- **Description**: Top-level organizational unit containing related chapters on a specific topic
- **Attributes**:
  - id: String (unique identifier: e.g., "module-1-foundations", "module-2-robotic-nervous-system", "module-3-ai-robot-brain")
  - name: String (display name: e.g., "Foundations", "Robotic Nervous System")
  - description: String (brief description of the module's focus)
  - order: Integer (sequential order in the book)
  - chapters: Array[Chapter] (chapters contained in this module)
  - learningObjectives: Array[String] (what students will learn)
  - prerequisites: Array[String] (required knowledge)
  - estimatedDuration: Integer (estimated time to complete in hours)
  - status: Enum ("draft", "review", "published", "deprecated")
  - createdAt: DateTime (creation timestamp)
  - updatedAt: DateTime (last update timestamp)
- **Relationships**:
  - Contains multiple Chapter entities
  - Connected to previous and next Module (for sequential navigation)
  - Associated with multiple Exercise entities

### 3. Chapter
- **Description**: Subdivision of a module containing focused content on a specific aspect
- **Attributes**:
  - id: String (unique identifier: e.g., "module-1-chapter-1", "module-2-chapter-3")
  - title: String (chapter title)
  - description: String (brief chapter description)
  - module: Module (parent module reference)
  - order: Integer (order within the parent module)
  - contentPath: String (path to the markdown file in docs/)
  - learningObjectives: Array[String] (specific learning objectives for this chapter)
  - prerequisites: Array[String] (knowledge required for this chapter)
  - estimatedReadingTime: Integer (reading time in minutes)
  - exercises: Array[Exercise] (exercises associated with this chapter)
  - examples: Array[Example] (code examples in this chapter)
  - solutions: Array[Solution] (solutions for exercises in this chapter)
  - status: Enum ("draft", "review", "published", "deprecated")
  - createdAt: DateTime (creation timestamp)
  - updatedAt: DateTime (last update timestamp)
- **Relationships**:
  - Belongs to one Module entity
  - Contains multiple Exercise, Example, and Solution entities
  - Connected to previous and next Chapter (for sequential navigation)
  - Associated with one Module

### 4. Exercise
- **Description**: Educational exercise for students to practice concepts learned
- **Attributes**:
  - id: String (unique identifier: e.g., "module-1-chapter-2-exercise-1")
  - title: String (exercise title)
  - description: String (detailed exercise description)
  - chapter: Chapter (parent chapter reference)
  - difficulty: Enum ("beginner", "intermediate", "advanced")
  - type: Enum ("theoretical", "practical", "coding", "analysis")
  - instructions: String (step-by-step instructions)
  - expectedOutcome: String (what the student should achieve)
  - hints: Array[String] (helpful hints for solving the exercise)
  - resources: Array[ResourceLink] (additional resources for the exercise)
  - status: Enum ("draft", "review", "published", "deprecated")
  - createdAt: DateTime (creation timestamp)
  - updatedAt: DateTime (last update timestamp)
- **Relationships**:
  - Belongs to one Chapter entity
  - Associated with one Solution entity
  - Connected to multiple ResourceLink entities

### 5. Example
- **Description**: Code or conceptual example illustrating key concepts
- **Attributes**:
  - id: String (unique identifier: e.g., "module-2-chapter-1-example-1")
  - title: String (example title)
  - description: String (brief description of the example)
  - chapter: Chapter (parent chapter reference)
  - code: String (actual code example, if applicable)
  - language: String (programming language, if code example)
  - explanation: String (explanation of the example)
  - type: Enum ("code", "conceptual", "diagram", "simulation")
  - tags: Array[String] (relevant tags for categorization)
  - status: Enum ("draft", "review", "published", "deprecated")
  - createdAt: DateTime (creation timestamp)
  - updatedAt: DateTime (last update timestamp)
- **Relationships**:
  - Belongs to one Chapter entity
  - May reference external ResourceLink entities

### 6. Solution
- **Description**: Solution to an exercise with detailed explanation
- **Attributes**:
  - id: String (unique identifier: e.g., "module-1-chapter-2-solution-1")
  - exercise: Exercise (reference to the associated exercise)
  - content: String (detailed solution and explanation)
  - alternativeApproaches: Array[String] (other possible approaches)
  - commonMistakes: Array[String] (mistakes students commonly make)
  - learningPoints: Array[String] (key learning points from the solution)
  - status: Enum ("draft", "review", "published", "deprecated")
  - createdAt: DateTime (creation timestamp)
  - updatedAt: DateTime (last update timestamp)
- **Relationships**:
  - Associated with one Exercise entity
  - Belongs to same Chapter as the exercise

### 7. NavigationLink
- **Description**: Navigation element in the site's navigation system
- **Attributes**:
  - id: String (unique identifier: e.g., "nav-read-book", "nav-about")
  - label: String (display text for the link)
  - url: String (destination URL)
  - type: Enum ("internal", "external", "anchor")
  - position: String ("top-bar", "sidebar", "footer", "dropdown")
  - icon: String (optional icon identifier)
  - order: Integer (display order in navigation)
  - isVisible: Boolean (whether the link is visible)
  - status: Enum ("active", "inactive", "hidden")
- **Relationships**:
  - May link to Module, Chapter, or external resource

### 8. EntryPoint
- **Description**: Primary access point for users to enter the documentation
- **Attributes**:
  - id: String (unique identifier: e.g., "entry-read-book", "entry-start-here")
  - label: String (button/link text)
  - destination: String (where the entry point leads)
  - style: String ("primary-button", "secondary-button", "link")
  - position: String ("homepage", "module-index", "chapter-intro")
  - description: String (what this entry point does)
  - status: Enum ("active", "inactive")
- **Relationships**:
  - Leads to Module or Chapter entities

### 9. ResourceLink
- **Description**: External or internal resource referenced in content
- **Attributes**:
  - id: String (unique identifier: e.g., "resource-nvidia-isaac-docs")
  - title: String (resource title)
  - url: String (resource URL)
  - type: Enum ("documentation", "tutorial", "paper", "video", "tool", "code")
  - description: String (brief description of the resource)
  - category: String ("essential", "supplementary", "advanced", "reference")
  - isExternal: Boolean (whether this is an external link)
  - status: Enum ("active", "broken", "deprecated")
  - createdAt: DateTime (creation timestamp)
  - updatedAt: DateTime (last update timestamp)
- **Relationships**:
  - Associated with multiple Exercise, Example, or Chapter entities

### 10. UserProgress
- **Description**: Tracks user progress through the book content (optional feature)
- **Attributes**:
  - id: String (unique identifier for the user's progress record)
  - userId: String (user identifier)
  - completedChapters: Array[Chapter] (chapters the user has completed)
  - currentChapter: Chapter (the chapter the user is currently reading)
  - timeSpent: Integer (total time spent in minutes)
  - exercisesCompleted: Array[Exercise] (exercises completed by the user)
  - lastAccessed: DateTime (when the user last accessed the content)
  - completionPercentage: Float (overall completion percentage)
  - status: Enum ("active", "paused", "completed")
  - createdAt: DateTime (creation timestamp)
  - updatedAt: DateTime (last update timestamp)
- **Relationships**:
  - Associated with multiple Chapter and Exercise entities
  - Belongs to one User (if user system is implemented)

## Entity Relationships

### Module ↔ Chapter Relationship
- **Type**: One-to-Many (One Module contains Many Chapters)
- **Description**: Each module contains multiple chapters that build upon the module's theme
- **Constraint**: Chapters must belong to exactly one module
- **Navigation**: Forward (module → chapters) and backward (chapter → module) navigation

### Chapter ↔ Exercise Relationship
- **Type**: One-to-Many (One Chapter contains Many Exercises)
- **Description**: Each chapter may contain multiple exercises for student practice
- **Constraint**: Exercises must belong to exactly one chapter
- **Navigation**: Exercises are listed at the end of each chapter

### Chapter ↔ Example Relationship
- **Type**: One-to-Many (One Chapter contains Many Examples)
- **Description**: Each chapter may contain multiple examples illustrating concepts
- **Constraint**: Examples must belong to exactly one chapter
- **Navigation**: Examples are embedded within chapter content

### Exercise ↔ Solution Relationship
- **Type**: One-to-One (One Exercise has One Solution)
- **Description**: Each exercise has exactly one corresponding solution
- **Constraint**: Each exercise must have exactly one solution
- **Navigation**: Solutions are linked from exercises

### NavigationLink ↔ Content Relationship
- **Type**: One-to-Many (One Navigation Link can point to Many Content Types)
- **Description**: Navigation links can point to modules, chapters, or external resources
- **Constraint**: Navigation links must have a valid destination
- **Navigation**: Primary navigation mechanism for users

## Validation Rules

1. **Module** entities must have a unique ID and non-empty name
2. **Chapter** entities must belong to a valid Module and have sequential ordering within the module
3. **Exercise** entities must belong to a valid Chapter and have a corresponding Solution
4. **Example** entities must belong to a valid Chapter and have appropriate content type
5. **Solution** entities must reference a valid Exercise
6. **NavigationLink** entities must have a valid URL and appropriate visibility settings
7. **EntryPoint** entities must have a valid destination and active status
8. **ResourceLink** entities must have valid URLs and appropriate categorization
9. **Homepage** entity must have proper metadata for SEO and accessibility
10. **UserProgress** entities (if implemented) must have valid user references and consistent state

## Access Patterns

### Read-Heavy Operations
- Homepage → Module → Chapter navigation (most frequent)
- Search functionality → Chapter lookup
- Breadcrumb navigation → Module/Chapter hierarchy

### Write Operations
- Content updates (infrequent, by maintainers)
- Progress tracking (moderate, by users)
- Exercise submissions (moderate, by users)

### Administrative Operations
- Module/Chapter creation and updates
- Navigation structure modifications
- User progress management