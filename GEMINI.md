# Gemini / Antigravity Home Assistant Development Guide

This file serves as a reference for future AI assistant sessions modifying or creating automations, scripts, and configuration files in this workspace.

---

## 📂 Key File Locations

- **Automations**: `/Volumes/config/automations.yaml` — Main automation file containing all event-driven logic and action sequences.
- **Scenes**: `/Volumes/config/scenes.yaml` — Scene configurations. Very useful for mapping entity IDs to their `friendly_name` (e.g., mapping `"Jude's Nightstand"` to `light.hue_color_lamp_1_3`).
- **Scripts**: `/Volumes/config/scripts.yaml` — Reusable sequence scripts.
- **Template Sensors**: `/Volumes/config/template_sensors.yaml` — Virtual template-based sensors and binary sensors.
- **General Configuration**: `/Volumes/config/configuration.yaml` — Core integration setup and platform inclusion.

---

## 🛠️ MCP Tool Usage

Use the `homeassistant` MCP server to interact with the Home Assistant instance:
- **`GetLiveContext`**: Check the current state of exposed entities. Filter using arguments:
  - `domain`: e.g. `light`, `sensor`, `switch`, `scene`
  - `name`: e.g. `Bedtime`, `Morning`
  - `area`: e.g. `Bedroom`, `Nursery`
- **Mapping Entities**: If entities aren't exposed to the Smart Speaker integration (causing `GetLiveContext` to return no results), map them by searching through `/Volumes/config/scenes.yaml` or `/Volumes/config/automations.yaml` for their friendly names.

---

## 🐍 YAML Validation Workflow

Before completing any modifications to `.yaml` files, validate the configuration files using a temporary Python virtual environment with `PyYAML` to avoid breaking Home Assistant syntax:

```bash
# 1. Create a temporary virtual environment
python3 -m venv /tmp/venv

# 2. Install PyYAML
/tmp/venv/bin/pip install pyyaml

# 3. Safely load the file to parse syntax errors
/tmp/venv/bin/python3 -c "import yaml; yaml.safe_load(open('/Volumes/config/automations.yaml'))"

# 4. Clean up the venv
rm -rf /tmp/venv
```

