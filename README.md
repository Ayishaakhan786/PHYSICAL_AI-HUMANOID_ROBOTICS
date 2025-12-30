# Physical AI & Humanoid Robotics - Spec-Driven Book

A comprehensive guide to Physical AI and Humanoid Robotics, built using Spec-Kit Plus workflow and Docusaurus.

## Overview

This book provides a complete learning path for understanding Physical AI (Embodied Intelligence) from fundamentals to advanced implementation. It covers:

- **Module 1**: Physical AI Foundations - What is Physical AI, embodied intelligence, digital vs physical AI
- **Module 2**: ROS 2 Fundamentals - Nodes, topics, services, URDF, rclpy
- **Module 3**: Digital Twin Simulation - Gazebo, Unity, sensors, environment modeling
- **Module 4**: NVIDIA Isaac Platform - GPU simulation, perception, VSLAM, Nav2
- **Module 5**: VLA & Conversational Robotics - Vision-Language-Action, LLMs, speech
- **Module 6**: Humanoid Robotics - Kinematics, locomotion, interaction
- **Hardware & Lab Architecture**: Workstations, Jetson, sensors, on-prem vs cloud
- **Sim-to-Real Challenges**: Domain randomization, reality gap, deployment constraints
- **Capstone**: Autonomous humanoid system design

## Features

- **Spec-Driven Development**: Built using Spec-Kit Plus workflow (spec → plan → tasks → implement)
- **High-Level Technical Content**: Conceptual explanations with system connections, no full codebases
- **Mermaid/ASCII Diagrams**: All architecture and flow diagrams use Markdown-compatible formats
- **6 Core Modules**: Structured learning path from foundations to advanced topics
- **Humanoid Case Studies**: Primary examples use humanoid robots with transferability notes
- **APA-Style Citations**: All factual claims traceable to authoritative sources
- **Docusaurus Platform**: Modern, responsive documentation site

## Tech Stack

- **Docusaurus**: v3.9.2 (Static site generator)
- **Node.js**: v24+ (Build system)
- **Markdown**: MDX supported for interactive content
- **Diagrams**: Mermaid.js for architecture visualization
- **Deploy**: GitHub Pages

## Getting Started

### Prerequisites

- **Node.js**: v20 or higher
- **Git**: For version control and deployment
- **Text Editor**: VS Code, or any Markdown editor

### Local Development

```bash
# Clone repository
git clone https://github.com/PhysicalAI-Humanoid-Robotics/PHYSICALAI_HUMANOID_ROBOTICS.git
cd PHYSICALAI_HUMANOID_ROBOTICS

# Navigate to website directory
cd website

# Install dependencies
npm install

# Start development server
npm start
```

Open http://localhost:3000 to view the book locally.

### Building for Production

```bash
# Build static site
cd website
npm run build

# Output: build/ directory
```

### Previewing Production Build

```bash
# Serve built site locally
cd website
npm run serve
```

## Project Structure

```
PHYSICALAI_HUMANOID_ROBOTICS/
├── website/                 # Docusaurus site
│   ├── docs/            # Book content (MDX)
│   │   ├── intro/       # Front matter
│   │   ├── modules/      # 6 core modules
│   │   ├── hardware/     # Hardware architecture
│   │   ├── sim-to-real/  # Sim-to-real challenges
│   │   └── capstone/     # Capstone system design
│   ├── docusaurus.config.js
│   ├── sidebars.js
│   └── src/
├── specs/                  # Spec-Kit Plus artifacts
│   └── 001-physical-ai-book/
│       ├── spec.md
│       ├── plan.md
│       ├── tasks.md
│       └── checklists/
├── .specify/              # Spec-Kit Plus templates
└── history/                # PHRs (Prompt History Records)
    └── prompts/
```

## Content Structure

Each chapter follows a consistent template:

1. **Learning Objectives**: 3-6 measurable outcomes
2. **Prerequisites**: Required knowledge or previous chapters
3. **Core Concepts**: System-level explanations
4. **Examples/Code Snippets**: Runnable Python/Ros2/simulation code
5. **System Connectivity**: Links to related modules and future topics
6. **Transferability Notes**: Application to other robot types
7. **Summary**: Key takeaways and next steps
8. **References**: APA-style citations

## Spec-Kit Plus Workflow

This project demonstrates Spec-Kit Plus methodology:

1. **Specification** (`specs/001-physical-ai-book/spec.md`)
   - User stories (6 prioritized stories)
   - Functional requirements (18 FRs)
   - Success criteria (13 SCs)

2. **Planning** (`specs/001-physical-ai-book/plan.md`)
   - Architecture decisions
   - Research findings
   - Data models
   - Quickstart guide
   - Quality gate contracts

3. **Tasks** (`specs/001-physical-ai-book/tasks.md`)
   - 144 tasks across 13 phases
   - User story organization
   - Dependency tracking

4. **Implementation** (`website/docs/`)
   - 59 chapters across 6 modules + hardware/sim-to-real/capstone
   - Mermaid diagrams for visualization
   - Code examples for each topic

5. **Validation**
   - Docusaurus build succeeds
   - All internal links resolve
   - Content follows template structure

## Deployment

This book is deployed to GitHub Pages:
- **URL**: https://physicalai-humanoid-robotics.github.io
- **Branch**: `001-physical-ai-book`
- **Workflow**: Automated on push to main branch

## Contributing

Contributions are welcome! To contribute:

1. Fork the repository
2. Create a feature branch
3. Make your changes following the chapter template
4. Ensure Docusaurus build passes: `npm run build`
5. Submit a pull request

## License

This project is licensed under the MIT License - see LICENSE file for details.

## Acknowledgments

- **Spec-Kit Plus**: Spec-driven development framework
- **Claude Code**: AI-assisted development
- **Docusaurus**: Static site generator
- **Open Robotics**: ROS 2 documentation and examples
- **NVIDIA**: Isaac Sim and Isaac ROS documentation
- **Mermaid**: Diagram rendering library

## References

- [Docusaurus Documentation](https://docusaurus.io/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaac-sim)
- [Spec-Kit Plus](https://github.com/your-org/spec-kit-plus)

---

Built with [Spec-Kit Plus](https://github.com/your-org/spec-kit-plus) + [Claude Code](https://claude.com/code)
