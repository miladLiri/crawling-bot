
# AI Crawling Bot 🐞

An educational crawling robot platform designed for AI students. This project provides everything needed to design, build, and program an autonomous crawling robot — including 3D models, firmware, logic templates, and full documentation.

---

## 📦 What’s Inside

- `template.ino`: A logic template students can customize to control the robot.
- 3D models in Rhino and export formats for viewing or printing.
- Firmware for the robot's microcontroller (e.g., Arduino or ESP).
- Guides for assembly, deployment, and usage.
- Diagrams and images for wiring and construction help.

---

## 🛠️ Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/YOUR_USERNAME/ai-crawling-bot.git
```

### 2. Open and Edit the Logic Template

Modify the file:

```bash
templates/template.ino
```

Use it to define how the robot should behave. You’ll write AI logic here.

### 3. Upload Logic and Firmware

Follow the steps in the [User Guide](docs/user-guide.md) to upload your logic and firmware to the robot.

---

## 🧱 Repository Structure

| Folder | Contents |
|--------|----------|
| `templates/` | Starter `.ino` logic file to customize |
| `models/` | 3D robot designs in Rhino (`.3dm`) and export formats (`.stl`, `.obj`) |
| `docs/` | Written guides, wiring diagrams, and usage instructions |
| `firmware/` | Microcontroller code (Arduino/ESP32/etc.) |

---

## 🖨️ 3D Model Exports

In the `models/` directory:
- `robot-design.3dm` — original Rhino model
- `exports/robot.stl`, `robot.obj` — printable and viewable 3D exports
- `exploded-view.png` — visual reference for assembly

---

## 📚 Documentation

Start with:

- [`docs/user-guide.md`](docs/user-guide.md): Robot setup, deployment, and control
- [`docs/installation.md`](docs/installation.md): Software installation and environment setup

Visual references are located in:

- `docs/images/`

---

## 📸 Preview

![Exploded View](docs/images/exploded-view.png)

---

## ⚖️ License

This project is licensed under the [MIT License](LICENSE). You are free to use, modify, and distribute this project for educational and non-commercial purposes.

---

## 🙋‍♀️ For Students

Students should:
- Fork or clone this repository.
- Modify `templates/template.ino` with their own logic.
- Deploy it to the crawling robot using the guides provided.
- Do **not** submit modified versions of this repository unless instructed.

---

## ❤️ Built for Education

This robot and its codebase were created as part of an AI course to help students learn real-world robotics, AI behavior programming, and system integration.

---
