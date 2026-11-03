# EARS-RL Checkpoints Directory

Stores neural network weights and policy checkpoints for system identification models
and RL policies (PPO / SAC / EARS-RL).

Each checkpoint must include:
1. Model state dictionary (`*.pt` or `*.safetensors`).
2. `manifest.json` containing random seed, Git commit hash, software version, and training hyperparameter config hash.

Model checkpoint binary files are excluded from Git via `.gitignore`.
