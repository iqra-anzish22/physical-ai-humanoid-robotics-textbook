# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

- Node.js (LTS version recommended)
- npm or yarn package manager
- Git
- A code editor (VS Code recommended)

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm start
   # or
   yarn start
   ```
   This command starts a local development server and opens the textbook in your browser. Most changes are reflected live without restarting the server.

4. **Build for production**
   ```bash
   npm run build
   # or
   yarn build
   ```
   This command generates static content into the `build` directory and can be served using any static hosting service.

## Adding New Content

### Creating a New Chapter

1. Create a new MDX file in the `docs/` directory or appropriate subdirectory
2. Add frontmatter with required metadata:
   ```md
   ---
   title: Chapter Title
   sidebar_position: 1
   description: Brief description of the chapter
   learning_outcomes:
     - Students will understand...
     - Students will be able to...
   ---
   ```

3. Write your content using MDX syntax
4. Add the new file to `sidebars.js` to make it appear in the navigation

### Adding Exercises

1. Use the custom Exercise component in your MDX files:
   ```mdx
   <Exercise
     title="Exercise Title"
     difficulty="beginner"
     type="practical"
   >
     Exercise content and instructions go here.
   </Exercise>
   ```

### Adding Diagrams

1. Place image files in the `static/img/` directory
2. Reference them in your MDX files:
   ```mdx
   ![Diagram Caption](/img/diagram-name.svg)
   ```

### Adding Learning Outcomes

1. Use the custom LearningOutcome component:
   ```mdx
   <LearningOutcome>
     - Students will understand the fundamental concepts of Physical AI
     - Students will be able to implement basic ROS 2 nodes
   </LearningOutcome>
   ```

## Internationalization (Urdu Translation)

1. To add Urdu translations, create a `i18n` directory in your project root
2. Run the following command to create Urdu locale files:
   ```bash
   npm run write-translations -- --locale ur
   ```

3. Edit the generated JSON files in `i18n/ur/` to add translations

## Running Tests

1. **Validate build**: Ensure the site builds without errors
   ```bash
   npm run build
   ```

2. **Check for broken links**:
   ```bash
   npm run serve
   # Then run link checker tools
   ```

3. **Verify responsive design**:
   - Test on different screen sizes
   - Use browser developer tools to simulate mobile devices

## Deployment

The textbook can be deployed to GitHub Pages or Vercel:

### GitHub Pages
1. Update `docusaurus.config.js` with your repository details
2. Run: `GIT_USER=<your-github-username> CURRENT_BRANCH=main USE_SSH=true npm run deploy`

### Vercel
1. Push your code to a Git repository
2. Import the project in Vercel
3. Set the build command to `npm run build`
4. Set the output directory to `build`

## Troubleshooting

- **Build errors**: Check that all MDX syntax is valid and all required frontmatter fields are present
- **Missing content**: Ensure all files are properly referenced in `sidebars.js`
- **Translation issues**: Verify that translation files follow the correct structure
- **Component errors**: Check that all custom components are properly imported in MDX files