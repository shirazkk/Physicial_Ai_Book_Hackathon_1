# Physical AI & Humanoid Robotics Book - Content Creation Guide

This guide explains how to add new modules and chapters to the Physical AI & Humanoid Robotics book.

## Adding New Modules

To add a new module:

1. Create a new directory in the `docs/` folder with a descriptive name:
   ```
   docs/module-3-new-module-name/
   ```

2. Add an introductory chapter file (e.g., `chapter1-introduction.md`) in the new module directory.

3. Update the `sidebars.ts` file to include the new module and its chapters in the navigation structure.

## Adding New Chapters

To add a new chapter to an existing module:

1. Create a new markdown file in the appropriate module directory:
   ```
   docs/module-name/chapterN-topic-name.md
   ```

2. Include the proper frontmatter at the top of the file:
   ```markdown
   ---
   title: Chapter Title
   description: Brief description of the chapter content
   ---
   ```

3. Follow the standard documentation format:
   - Overview
   - Learning Objectives
   - Theory
   - Examples (with code snippets)
   - Exercises
   - Solutions
   - Summary
   - Further Reading

4. Update the `sidebars.ts` file to include the new chapter in the navigation.

## Standard Documentation Format

All chapters should follow this structure:

```markdown
---
title: Chapter Title
description: Brief description
---

# Chapter Title

## Overview
Brief overview of the chapter.

## Learning Objectives
- Objective 1
- Objective 2

## Theory
Detailed theoretical concepts.

## Examples
Practical examples with code.

## Exercises
Practice problems.

## Solutions
Solutions to exercises.

## Summary
Key takeaways.

## Further Reading
Additional resources.
```

## Navigation Consistency

When adding new content, ensure navigation consistency by:
- Following the same naming conventions
- Maintaining the hierarchical structure (Module → Chapter → Section)
- Including proper links to previous/next chapters where applicable
- Using consistent formatting and styling