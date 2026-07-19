# LeRobot CUDA Training Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `uv sync` install CUDA 13.0 PyTorch on Windows/Linux and verify ACT training runs on the RTX 3060.

**Architecture:** Pin PyTorch 2.11 and torchvision 0.26 as direct project dependencies, then route only those packages through an explicit official CUDA 13.0 index on Windows/Linux. Keep PyAV as the CPU video decoder and move only the ACT policy computation to CUDA.

**Tech Stack:** Python 3.12, uv, PyTorch 2.11, torchvision 0.26, CUDA 13.0 wheels, LeRobot 0.6, PyAV, NVIDIA RTX 3060

## Global Constraints

- Windows/Linux must use CUDA 13.0 PyTorch by default after a plain `uv sync`.
- Pin `torch==2.11.0` and `torchvision==0.26.0`.
- Use `https://download.pytorch.org/whl/cu130` through an explicit uv index.
- macOS must fall back to PyPI and is not expected to support CUDA training.
- Keep `--dataset.video_backend=pyav`; do not add TorchCodec/NVDEC setup.
- Do not install a local CUDA Toolkit; rely on the NVIDIA driver and wheel-bundled CUDA runtime.
- Verification must not push a model or save a checkpoint.
- Use the existing dataset `OJII3/so101_dataset_20260718_171048`.

---

### Task 1: Resolve CUDA PyTorch through uv

**Files:**
- Modify: `lerobot/pyproject.toml`
- Modify: `lerobot/uv.lock`

**Interfaces:**
- Consumes: NVIDIA driver 591.86 and the official `pytorch-cu130` package index.
- Produces: a reproducible `uv` environment where `torch.__version__ == "2.11.0+cu130"`, `torch.version.cuda == "13.0"`, and CUDA device 0 is the RTX 3060.

- [ ] **Step 1: Run the failing CUDA environment check**

Run from `lerobot/`:

```powershell
uv run python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available()); assert torch.__version__ == '2.11.0+cu130'; assert torch.version.cuda == '13.0'; assert torch.cuda.is_available()"
```

Expected before the change: FAIL because the current output is `2.11.0+cpu None False`.

- [ ] **Step 2: Add the direct PyTorch dependencies and CUDA index**

Update `lerobot/pyproject.toml` to this complete content:

```toml
[project]
name = "so-arm-lerobot"
version = "0.1.0"
description = "LeRobot CLI environment for SO-101 leader/follower sync and ACT"
requires-python = ">=3.12"
dependencies = [
    "lerobot[core_scripts,training,feetech]",
    "torch==2.11.0",
    "torchvision==0.26.0",
]

[tool.uv]
package = false

[tool.uv.sources]
torch = [
    { index = "pytorch-cu130", marker = "sys_platform == 'linux' or sys_platform == 'win32'" },
]
torchvision = [
    { index = "pytorch-cu130", marker = "sys_platform == 'linux' or sys_platform == 'win32'" },
]

[[tool.uv.index]]
name = "pytorch-cu130"
url = "https://download.pytorch.org/whl/cu130"
explicit = true
```

- [ ] **Step 3: Resolve and install the CUDA environment**

Run from `lerobot/`:

```powershell
uv lock
uv sync
```

Expected: both commands exit 0; `uv.lock` records `torch==2.11.0+cu130` and `torchvision==0.26.0+cu130` from the `pytorch-cu130` index.

- [ ] **Step 4: Run the CUDA environment check again**

```powershell
uv run python -c "import torch; print('torch=', torch.__version__); print('cuda_runtime=', torch.version.cuda); print('available=', torch.cuda.is_available()); print('device=', torch.cuda.get_device_name(0)); assert torch.__version__ == '2.11.0+cu130'; assert torch.version.cuda == '13.0'; assert torch.cuda.is_available(); assert torch.cuda.get_device_name(0) == 'NVIDIA GeForce RTX 3060'"
```

Expected: PASS with `torch=2.11.0+cu130`, `cuda_runtime=13.0`, `available=True`, and `device=NVIDIA GeForce RTX 3060`.

- [ ] **Step 5: Verify a real CUDA tensor operation and memory allocation**

```powershell
uv run python -c "import torch; x=torch.ones((1024,1024), device='cuda'); y=x @ x; torch.cuda.synchronize(); allocated=torch.cuda.memory_allocated(); print('result=', y[0,0].item()); print('allocated_bytes=', allocated); assert y.device.type == 'cuda'; assert y[0,0].item() == 1024.0; assert allocated > 0"
```

Expected: PASS with `result=1024.0` and `allocated_bytes` greater than zero.

- [ ] **Step 6: Check the lockfile and diff**

```powershell
uv lock --check
git diff --check
git diff -- pyproject.toml uv.lock
```

Expected: both checks exit 0; the diff contains only the dependency declarations, CUDA index metadata, and lockfile resolution changes.

- [ ] **Step 7: Commit the CUDA dependency configuration**

```powershell
git add pyproject.toml uv.lock
git commit -m "build(lerobot): use CUDA PyTorch wheels"
```

### Task 2: Document and smoke-test CUDA ACT training

**Files:**
- Modify: `lerobot/README.md`

**Interfaces:**
- Consumes: Task 1's CUDA-enabled `uv` environment and the existing `DATASET_REPO_ID`/PyAV training command.
- Produces: copyable CUDA verification instructions and evidence that one ACT optimizer step completes with `policy.device=cuda`.

- [ ] **Step 1: Run the failing README documentation check**

Run from `lerobot/`:

```powershell
$readme = Get-Content -Raw README.md
if ($readme.Contains('torch.cuda.is_available()')) { throw 'README already contains the CUDA check' }
Write-Output 'RED: README does not document CUDA verification'
exit 1
```

Expected before the change: exit 1 with `RED: README does not document CUDA verification`.

- [ ] **Step 2: Add CUDA setup verification to the README**

Insert the following after the setup code block in `lerobot/README.md`:

````markdown
### CUDA の確認

Windows/Linux では `uv sync` が CUDA 13.0 版 PyTorch を導入する。学習前に GPU が認識されていることを確認する:

```bash
uv run python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

この環境での期待値は `2.11.0+cu130`、`True`、`NVIDIA GeForce RTX 3060`。
````

- [ ] **Step 3: Verify the README contains the CUDA check once**

```powershell
$matchingLines = @(Select-String -LiteralPath README.md -Pattern 'torch.cuda.is_available')
if ($matchingLines.Count -ne 1) { throw "Expected one CUDA command line, found $($matchingLines.Count)" }
Write-Output 'README CUDA check: PASS'
```

Expected: `README CUDA check: PASS`.

- [ ] **Step 4: Run one ACT training step on CUDA**

```powershell
uv run lerobot-train --policy.type=act --policy.repo_id=OJII3/act_so101 --policy.push_to_hub=false --dataset.repo_id=OJII3/so101_dataset_20260718_171048 --dataset.video_backend=pyav --output_dir=outputs/train/cuda_smoke --policy.device=cuda --steps=1 --num_workers=0 --save_checkpoint=false --log_freq=1
```

Expected: the printed configuration retains `'device': 'cuda'`, no `Switching to 'cpu'` warning appears, the progress reaches `1/1`, and the final log contains `End of training`.

- [ ] **Step 5: Confirm the smoke test did not leave training artifacts**

```powershell
if (Test-Path -LiteralPath 'outputs/train/cuda_smoke') { throw 'Unexpected CUDA smoke artifacts remain' }
Write-Output 'CUDA smoke artifacts: none'
```

Expected: `CUDA smoke artifacts: none`.

- [ ] **Step 6: Run final documentation and repository checks**

```powershell
uv lock --check
git diff --check
git status --short --branch
```

Expected: `uv lock --check` and `git diff --check` exit 0; only `README.md` is uncommitted at this task boundary.

- [ ] **Step 7: Commit the CUDA documentation**

```powershell
git add README.md
git commit -m "docs(lerobot): document CUDA verification"
```

### Task 3: Publish the completed branch

**Files:**
- No file changes.

**Interfaces:**
- Consumes: the verified commits from Tasks 1 and 2.
- Produces: an updated remote branch and Draft PR with CUDA dependency, validation, and ACT smoke-test details.

- [ ] **Step 1: Run final verification from `lerobot/`**

```powershell
uv lock --check
uv run python -c "import torch; assert torch.__version__ == '2.11.0+cu130'; assert torch.version.cuda == '13.0'; assert torch.cuda.is_available(); assert torch.cuda.get_device_name(0) == 'NVIDIA GeForce RTX 3060'; print('CUDA environment: PASS')"
git diff HEAD~2 HEAD --check
git status --short --branch
```

Expected: CUDA environment prints `PASS`, the two-commit diff check exits 0, and the worktree is clean.

- [ ] **Step 2: Push the branch**

```powershell
git push
```

Expected: `codex/fix-lerobot-train-command` is updated on `origin`.

- [ ] **Step 3: Update Draft PR #38**

Set the title to `build(lerobot): enable CUDA ACT training` and update the body to include:

```markdown
## CUDA validation

- Installed `torch==2.11.0+cu130` and `torchvision==0.26.0+cu130` through uv.
- Confirmed `torch.cuda.is_available()` is `True` on `NVIDIA GeForce RTX 3060`.
- Completed one ACT training step with `policy.device=cuda` and PyAV video decoding.
```

Expected: Draft PR #38 points to the pushed head commit and includes the CUDA validation evidence.
