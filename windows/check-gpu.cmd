@echo off
rem Isaac Sim viability check (architecture.md open item): GPU model + VRAM.
echo == GPU ==
wmic path win32_VideoController get name,adapterram /format:list
echo.
echo == NVIDIA driver (if any) ==
where nvidia-smi >nul 2>&1 && nvidia-smi --query-gpu=name,memory.total --format=csv || echo no nvidia-smi in PATH
echo.
echo Isaac Sim needs: NVIDIA RTX-class GPU, driver >= 535, and 16 GB+ RAM.
pause
