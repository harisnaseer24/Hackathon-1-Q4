****# Physical AI & Humanoid Robotics — Spec-Driven Book + RAG System

This repository contains a **spec-driven technical book** on *Physical AI and Humanoid Robotics*, built using **Docusaurus** and authored with **Spec-Kit Plus** and **Claude Code**.  
The project also integrates a **Retrieval-Augmented Generation (RAG) chatbot** that answers questions directly from the book content.

The goal is to bridge **AI cognition and physical embodiment**, guiding learners from robotics middleware to vision-language-action systems.

---

## 📘 Project Overview

This project delivers:

- A modular, Docusaurus-based technical book
- Structured using **spec-first design (`sp.specify`, `sp.plan`)**
- Focused on **humanoid robotics and Physical AI**
- An embedded **RAG chatbot** for contextual Q&A
- Fully reproducible and deployable via GitHub Pages

---

## 🧠 Course Structure

The book is organized into four modules, each with focused chapters written in `.md` format.

### Module 1 — The Robotic Nervous System (ROS 2)
- ROS 2 architecture and middleware
- Nodes, topics, and services
- Python agents with `rclpy`
- Humanoid modeling using URDF

### Module 2 — The Digital Twin (Gazebo & Unity)
- Digital twin concepts for robotics
- Physics simulation with Gazebo
- Sensor simulation (LiDAR, depth, IMU)
- ROS 2 integration

### Module 3 — The AI-Robot Brain (NVIDIA Isaac™)
- Isaac Sim and Isaac ROS overview
- Synthetic data and perception pipelines
- Visual SLAM and navigation

### Module 4 — Vision-Language-Action (VLA)
- VLA system architecture
- Voice-to-action pipelines using Whisper
- LLM-based goal decomposition and planning
- End-to-end autonomous humanoid behavior

---

## 🧱 Technology Stack

### Documentation & Authoring
- **Docusaurus**
- Markdown (`.md`) only
- Sidebar-based modular navigation

### AI & Robotics
- ROS 2
- Gazebo
- Unity (conceptual integration)
- NVIDIA Isaac Sim / Isaac ROS
- OpenAI Whisper
- Vision-Language Models (LLM-based planning)

### RAG System
- OpenAI Agents / ChatKit SDKs
- FastAPI backend
- Qdrant (vector database)
- Neon Serverless PostgreSQL
- Retrieval strictly grounded in book content

---

## 📁 Repository Structure (High-Level)

****
