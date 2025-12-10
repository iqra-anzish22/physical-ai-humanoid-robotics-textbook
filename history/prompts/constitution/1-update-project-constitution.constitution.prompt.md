---
id: 1
title: Update-Project-Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "setup", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Update Project Constitution

## Prompt Text

```
Project: Physical AI & Humanoid Robotics Textbook (Part 1)
Mode: Spec-Driven Book Creation | Frameworks: Docusaurus • Claude Code • Spec-Kit Plus

Purpose: Write, structure, and deploy a complete, student-friendly textbook. Part-1 = book only; Part-2 = AI/chat features.

Core Principles:
- Clear, beginner-friendly explanations
- Consistent chapter structure & learning outcomes
- Verified robotics content, APA citations
- Modular & expandable for Part-2

Scope:
- Build & deploy Docusaurus book
- Chapters: Physical AI intro, ROS 2, Gazebo & Unity, NVIDIA Isaac, Humanoid robotics, VLA basics
- Use Claude Code + Spec-Kit Plus
- Optional Urdu translation button

Out of Scope (Part-2):
- RAG chatbot, backend, authentication, personalization, selected-text QA, subagents

Constraints:
- Clean compilation, mobile-friendly, strict chapter order
- Urdu button must not break MDX
- No backend code

Governance:
- /sp.plan → outline | /sp.spec → chapter requirements | /sp.tasks → steps
- Constitution overrides all files

Workflows:
- Research + writing concurrent
- Gradual concept explanations
- Diagrams/examples/code snippets
- Clean Docusaurus UI

Quality Validation:
- Full module coverage, accurate content
- Consistent chapters, zero build errors
- Successful deployment

Success Criteria:
1. Book written & deployed
2. All modules readable
3. Navigation/layout works
4. Urdu button works
5. Public GitHub + live link

End of Constitution
```

## Response Text

Updated the project constitution file `.specify/memory/constitution.md` to reflect the Physical AI & Humanoid Robotics Textbook project requirements. The constitution now includes specific principles for educational content creation, Docusaurus best practices, and quality standards for the textbook project.

## Outcome

Successfully created a new constitution that replaces all placeholder values with specific principles and constraints for the Physical AI & Humanoid Robotics Textbook project. The constitution includes six core principles focused on educational content quality, consistent structure, verified technical content, modular design, Docusaurus best practices, and quality standards.

## Evaluation

The updated constitution aligns with the project requirements and includes governance rules specific to textbook development. All placeholder tokens have been replaced with concrete values reflecting the project's educational and technical goals.