---
id: 1
title: Clarity hardware software versions
stage: simclarify
date: 2026-01-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: 004-isaac-ai-brain
branch: 004-isaac-ai-brain
user: Claude Code
command: clarify
labels: ["clarification","hardware","software","simulation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-isaac-ai-brain/spec.md
tests:
 - null
---

## Prompt

lets clarify the specs for module 3 issac-ai-brain

## Response snapshot

This is the most critical component. NVIDIA Isaac Sim is an Omniverse application that requires "RTX" (Ray Tracing) capabilities. Standard laptops (MacBooks or non-RTX Windows machines) will not work. GPU (The Bottleneck): NVIDIA RTX 4070 Ti (12GB VRAM) or higher. Why: You need high VRAM to load the USD (Universal Scene Description) assets for the robot and environment, plus run the VLA (Vision-Language-Action) models simultaneously. Ideal: RTX 3090 or 4090 (24GB VRAM) allows for smoother "Sim-to-Real" training. CPU: Intel Core i7 (13th Gen+) or AMD Ryzen 9. Why: Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive. RAM: 64 GB DDR5 (32 GB is the absolute minimum, but will crash during complex scene rendering). OS: Ubuntu 22.04 LTS. Note: While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux. Dual-booting or dedicated Linux machines are mandatory for a friction-free experience.

A

A

## Outcome

- ✅ Impact: Critical hardware and software requirements clarified for Isaac Sim, enabling proper setup for students
- 🧪 Tests: null
- 📁 Files: Updated specs/004-isaac-ai-brain/spec.md with clarifications section
- 🔁 Next prompts: Additional clarifications may be needed for other aspects of the Isaac ecosystem
- 🧠 Reflection: Hardware requirements are critical for Isaac Sim functionality, and proper OS setup is essential for ROS 2 compatibility

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null