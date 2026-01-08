# Quickstart Guide: Docusaurus-based Frontend for Physical AI & Humanoid Robotics Book

## Overview
This guide provides a quick start for content creators and maintainers of the Physical AI & Humanoid Robotics book Docusaurus frontend. It covers the essential steps to set up, configure, and begin adding content to the documentation site.

## Prerequisites
- **Node.js**: Version 18.x or higher installed
- **npm or yarn**: Package manager for dependencies
- **Git**: Version control system
- **Operating System**: Linux, macOS, or Windows with WSL2
- **Hardware**: Minimum 8GB RAM (16GB+ recommended for development)

## Initial Setup

### 1. Clone and Initialize the Repository
```bash
# Clone the repository (if not already done)
git clone https://github.com/[your-org]/[your-repo-name].git
cd [your-repo-name]

# Install dependencies
npm install
# OR
yarn install
```

### 2. Verify Docusaurus Installation
```bash
# Check if Docusaurus is properly installed
npx docusaurus --version

# Verify the project structure
ls -la
# You should see:
# - docs/ directory with module folders
# - docusaurus.config.js
# - sidebars.js
# - package.json
# - src/ directory
# - static/ directory
```

### 3. Start Local Development Server
```bash
# Start the development server
npm run start
# OR
yarn start

# The site will be available at http://localhost:3000
# Changes to content will hot-reload automatically
```

## Content Structure

### Module Organization
The book content is organized in the `docs/` directory following this structure:
```
docs/
├── intro/                          # Introduction module
├── module-1-foundations/           # Module 1: Foundations of Physical AI
├── module-2-robotic-nervous-system/ # Module 2: Robotic Nervous System with ROS 2
├── module-3-ai-robot-brain/        # Module 3: AI-Robot Brain with Isaac Ecosystem
└── ...                            # Additional modules as needed
```

### Adding New Modules
To add a new module:
1. Create a new directory in `docs/` (e.g., `docs/module-4-advanced-topics/`)
2. Add an `index.md` file with module introduction
3. Create chapter files (e.g., `chapter-1-introduction.md`, `chapter-2-advanced-concepts.md`)
4. Update `sidebars.js` to include the new module in navigation

### Adding New Chapters
To add a new chapter to an existing module:
1. Create a new markdown file in the appropriate module directory
2. Use proper frontmatter (see below)
3. Update the module's sidebar configuration if needed

## Content Creation Guidelines

### Frontmatter Metadata
Each markdown file should include proper frontmatter at the top:
```markdown
---
sidebar_position: 1
description: "Brief description of the chapter content for SEO"
keywords: ["keyword1", "keyword2", "keyword3"]
---

# Chapter Title

Content goes here...
```

### Navigation Configuration
Update `sidebars.js` to control the navigation hierarchy:
```javascript
module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: Foundations',
      items: [
        'module-1-foundations/intro',
        'module-1-foundations/chapter-1-basic-concepts',
        'module-1-foundations/chapter-2-advanced-topics',
      ],
    },
    // Add more modules as needed
  ],
};
```

### Creating Educational Content
When creating educational content, follow this structure:
1. **Learning Objectives**: Clear, measurable objectives at the beginning
2. **Prerequisites**: What knowledge is required
3. **Theory Section**: Core concepts and explanations
4. **Examples**: Practical examples with code or diagrams
5. **Exercises**: Practice problems for students
6. **Solutions**: Solutions to exercises (in separate file or collapsible section)
7. **Summary**: Key takeaways and what's next

## Development Commands

### Common Commands
```bash
# Start development server
npm run start

# Build static site for production
npm run build

# Serve built site locally for testing
npm run serve

# Deploy to GitHub Pages (if configured)
npm run deploy

# Run tests
npm test

# Run ESLint to check code quality
npm run lint

# Format code with Prettier
npm run format
```

### Configuration Files
- `docusaurus.config.js`: Main site configuration (title, description, theme, plugins)
- `sidebars.js`: Navigation structure and hierarchy
- `src/css/custom.css`: Custom styling overrides
- `static/`: Static assets (images, documents, etc.)

## Custom Components

### Using Educational Components
The site includes custom components for educational content:
- **Admonitions**: For notes, tips, warnings, and important information
  ```markdown
  :::note
  This is a helpful note for students.
  :::

  :::tip
  This is a helpful tip.
  :::

  :::caution
  This is a warning about potential issues.
  :::

  :::danger
  This is an important danger notice.
  :::
  ```

### Code Examples
For code examples, use fenced code blocks with appropriate language identifiers:
````markdown
```python
# Python code example
def example_function():
    print("Hello, Physical AI!")
    return True
```

```bash
# Bash/command example
npm run start
```

```javascript
// JavaScript example
const greeting = "Hello, Docusaurus!";
console.log(greeting);
```
````

## Theming and Styling

### Custom Styles
To add custom styles:
1. Modify `src/css/custom.css` for global styles
2. Create custom React components in `src/components/` for reusable UI elements
3. Use CSS modules for component-specific styling

### Branding
The site uses the Physical AI & Humanoid Robotics branding with:
- Primary color: Set in theme configuration
- Typography: Professional, readable fonts
- Spacing: Consistent padding and margins
- Responsive design: Mobile-first approach

## Building and Deployment

### Local Build
```bash
# Build the site locally
npm run build

# The built site will be in the build/ directory
# You can serve it locally to test
npm run serve
```

### Deployment Options
1. **GitHub Pages**: Use `npm run deploy` (requires proper configuration)
2. **Netlify**: Deploy from the `build/` directory
3. **Vercel**: Deploy the entire project
4. **Static hosting**: Upload the `build/` directory contents to any static host

### Deployment Configuration
Check your `docusaurus.config.js` for deployment settings:
```javascript
// Example GitHub Pages configuration
module.exports = {
  // ...
  url: 'https://your-username.github.io',
  baseUrl: '/your-repository-name/',
  organizationName: 'your-username',
  projectName: 'your-repository-name',
  deploymentBranch: 'gh-pages',
  // ...
};
```

## Troubleshooting

### Common Issues
- **Port already in use**: Use `npm run start -- --port 3001` to use a different port
- **Module not showing in sidebar**: Verify the path in `sidebars.js` matches the file location
- **Images not displaying**: Place images in `static/img/` and reference with `/img/image-name.jpg`
- **Build errors**: Run `npm run clear` to clear Docusaurus cache
- **Hot reload not working**: Restart the development server

### Performance Tips
- Use compressed images (WebP format preferred)
- Keep markdown files focused (split large chapters if needed)
- Use appropriate heading hierarchy (H1 for page title, H2 for sections, etc.)
- Add loading states for large components

## Next Steps
1. Begin adding content to the existing module structure
2. Customize the theme to match your branding requirements
3. Configure search and analytics as needed
4. Set up deployment pipeline for production
5. Test accessibility compliance with automated tools

## Resources
- [Docusaurus Official Documentation](https://docusaurus.io/docs)
- [Markdown Guide](https://www.markdownguide.org/)
- [React Component Documentation](https://react.dev/learn)
- [Accessibility Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)