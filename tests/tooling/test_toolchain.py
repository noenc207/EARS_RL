"""Pytest suite for EARS-RL toolchain, MCP configuration, and Agent Skills."""

import json
import os
import re
from pathlib import Path

import pytest

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
SKILLS_DIR = WORKSPACE_ROOT / ".agents" / "skills"
GLOBAL_MCP_CONFIG = Path(os.environ.get("GEMINI_CONFIG_PATH", Path.home() / ".gemini" / "config" / "mcp_config.json"))
WORKSPACE_MCP_CONFIG = WORKSPACE_ROOT / ".mcp.json"

EXPECTED_SKILLS = [
    "ears-rl-architecture",
    "python-research-engineering",
    "actuator-system-identification",
    "reproducible-ml-experiments",
    "rl-sim-to-real",
    "robotics-hardware-safety",
    "test-driven-debugging",
    "dependency-security-audit",
]


def test_global_mcp_config_valid_json():
    """Verify global mcp_config.json exists and is valid JSON."""
    if not GLOBAL_MCP_CONFIG.exists():
        pytest.skip(f"Global MCP config not found at {GLOBAL_MCP_CONFIG} (expected in developer environment, optional in clean CI)")
    with open(GLOBAL_MCP_CONFIG, "r", encoding="utf-8") as f:
        data = json.load(f)
    assert "mcpServers" in data
    assert "context7" in data["mcpServers"]
    assert "filesystem" in data["mcpServers"]


def test_workspace_mcp_config_valid_json():
    """Verify workspace .mcp.json exists and is valid JSON."""
    if not WORKSPACE_MCP_CONFIG.exists():
        pytest.skip(f"Workspace .mcp.json not found at {WORKSPACE_MCP_CONFIG} (private local configuration)")
    with open(WORKSPACE_MCP_CONFIG, "r", encoding="utf-8") as f:
        data = json.load(f)
    assert "mcpServers" in data
    assert "context7" in data["mcpServers"]
    assert "filesystem" in data["mcpServers"]


def test_filesystem_mcp_is_strictly_scoped():
    """Verify filesystem MCP is strictly constrained to workspace root."""
    if not GLOBAL_MCP_CONFIG.exists():
        pytest.skip(f"Global MCP config not found at {GLOBAL_MCP_CONFIG}")
    with open(GLOBAL_MCP_CONFIG, "r", encoding="utf-8") as f:
        data = json.load(f)
    fs_args = data["mcpServers"]["filesystem"]["args"]
    target_path = Path(fs_args[-1]).resolve()
    assert target_path == WORKSPACE_ROOT, f"Filesystem MCP path {target_path} is not scoped to {WORKSPACE_ROOT}"


@pytest.mark.parametrize("skill_name", EXPECTED_SKILLS)
def test_skill_structure_and_frontmatter(skill_name: str):
    """Verify each skill has valid YAML frontmatter and proper name."""
    if not SKILLS_DIR.exists():
        pytest.skip(f"Skills directory not found at {SKILLS_DIR} (private local skills)")
    skill_file = SKILLS_DIR / skill_name / "SKILL.md"
    assert skill_file.exists(), f"Skill file missing: {skill_file}"
    content = skill_file.read_text(encoding="utf-8")

    match = re.match(r"^---\s*\n(.*?)\n---\s*\n(.*)$", content, re.DOTALL)
    assert match is not None, f"YAML frontmatter missing in {skill_name}"

    frontmatter = match.group(1)
    name_match = re.search(r"^name:\s*([\w\-]+)", frontmatter, re.MULTILINE)
    desc_match = re.search(r"^description:\s*(?:>-\s*)?(.*)", frontmatter, re.MULTILINE)

    assert name_match is not None, f"Missing 'name' field in {skill_name}"
    assert name_match.group(1).strip() == skill_name, f"Name mismatch: {name_match.group(1)} != {skill_name}"
    assert desc_match is not None, f"Missing 'description' field in {skill_name}"
    assert len(desc_match.group(1).strip()) > 10, f"Description too short in {skill_name}"


def test_hardware_safety_default():
    """Verify hardware_enabled is false by default in .env.example."""
    env_example = WORKSPACE_ROOT / ".env.example"
    assert env_example.exists()
    content = env_example.read_text(encoding="utf-8")
    assert "EARS_RL_HARDWARE_ENABLED=false" in content


def test_no_secrets_in_repo():
    """Ensure no hardcoded tokens or API keys exist in repository files."""
    patterns = [
        re.compile(r"ghp_[a-zA-Z0-9]{36}"),
        re.compile(r"github_pat_[a-zA-Z0-9_]{82}"),
        re.compile(r"sk-[a-zA-Z0-9]{48}"),
        re.compile(r"AKIA[0-9A-Z]{16}"),
    ]
    for path in WORKSPACE_ROOT.rglob("*"):
        if path.is_file() and not any(part.startswith(".git") for part in path.parts):
            if path.suffix in (".py", ".json", ".yaml", ".yml", ".md", ".toml", ".txt", ".example"):
                text = path.read_text(encoding="utf-8", errors="ignore")
                for p in patterns:
                    assert not p.search(text), f"Potential secret leak in {path}"
