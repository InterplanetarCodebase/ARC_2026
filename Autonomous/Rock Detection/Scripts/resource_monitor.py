from __future__ import annotations

import os
import sys
import time
from typing import Optional


class _StderrFilter:
    def __init__(self, wrapped) -> None:
        self._wrapped = wrapped
        self._drop_substrings = (
            "QFontDatabase: Cannot find font directory",
            "Note that Qt no longer ships fonts.",
        )

    def write(self, data):
        if any(token in data for token in self._drop_substrings):
            return len(data)
        return self._wrapped.write(data)

    def flush(self):
        return self._wrapped.flush()

    def isatty(self):
        return self._wrapped.isatty()

    @property
    def encoding(self):
        return getattr(self._wrapped, "encoding", "utf-8")


def configure_opencv_runtime() -> None:
    """Reduce noisy OpenCV/Qt logs while keeping GUI windows functional."""
    os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
    os.environ.setdefault("QT_LOGGING_RULES", "*.debug=false;qt.qpa.*=false;qt.fonts.*=false")
    if "QT_QPA_FONTDIR" not in os.environ:
        for candidate in (
            "/usr/share/fonts/truetype/dejavu",
            "/usr/share/fonts/truetype",
            "/usr/share/fonts",
        ):
            if os.path.isdir(candidate):
                os.environ["QT_QPA_FONTDIR"] = candidate
                break

    # Keep logs clean for repeated Qt font path warnings emitted by some cv2 builds.
    if not isinstance(sys.stderr, _StderrFilter):
        sys.stderr = _StderrFilter(sys.stderr)


try:
    import psutil  # type: ignore
except Exception:  # pragma: no cover
    psutil = None


try:
    import pynvml  # type: ignore
except Exception:  # pragma: no cover
    pynvml = None


class ResourceMonitor:
    def __init__(self, update_interval_s: float = 0.5) -> None:
        self.update_interval_s = update_interval_s
        self._last_sample_time = 0.0

        self._cpu_samples: list[float] = []
        self._gpu_util_samples: list[float] = []
        self._gpu_mem_samples_mb: list[float] = []

        self._latest_cpu: Optional[float] = None
        self._latest_ram_mb: Optional[float] = None
        self._latest_gpu_util: Optional[float] = None
        self._latest_gpu_mem_mb: Optional[float] = None

        self._pid = os.getpid()
        self._proc = psutil.Process(self._pid) if psutil is not None else None
        if self._proc is not None:
            self._proc.cpu_percent(interval=None)

        self._nvml_ready = False
        self._nvml_handles = []
        if pynvml is not None:
            try:
                pynvml.nvmlInit()
                self._nvml_ready = True
                for idx in range(pynvml.nvmlDeviceGetCount()):
                    self._nvml_handles.append(pynvml.nvmlDeviceGetHandleByIndex(idx))
            except Exception:
                self._nvml_ready = False
                self._nvml_handles = []

    def _sample_cpu(self) -> None:
        if self._proc is None:
            return
        try:
            self._latest_cpu = float(self._proc.cpu_percent(interval=None))
            mem_info = self._proc.memory_info()
            self._latest_ram_mb = float(mem_info.rss) / (1024.0 * 1024.0)
            self._cpu_samples.append(self._latest_cpu)
        except Exception:
            pass

    def _sample_gpu(self) -> None:
        if not self._nvml_ready:
            return

        gpu_utils: list[float] = []
        proc_gpu_mem_mb = 0.0

        for handle in self._nvml_handles:
            try:
                util = pynvml.nvmlDeviceGetUtilizationRates(handle)
                gpu_utils.append(float(util.gpu))
            except Exception:
                pass

            try:
                procs = pynvml.nvmlDeviceGetComputeRunningProcesses_v2(handle)
            except Exception:
                try:
                    procs = pynvml.nvmlDeviceGetComputeRunningProcesses(handle)
                except Exception:
                    procs = []

            for p in procs:
                try:
                    if int(p.pid) == self._pid and p.usedGpuMemory is not None:
                        proc_gpu_mem_mb += float(p.usedGpuMemory) / (1024.0 * 1024.0)
                except Exception:
                    continue

        if gpu_utils:
            self._latest_gpu_util = sum(gpu_utils) / len(gpu_utils)
            self._gpu_util_samples.append(self._latest_gpu_util)
        self._latest_gpu_mem_mb = proc_gpu_mem_mb
        self._gpu_mem_samples_mb.append(proc_gpu_mem_mb)

    def sample(self, force: bool = False) -> None:
        now = time.time()
        if not force and (now - self._last_sample_time) < self.update_interval_s:
            return
        self._last_sample_time = now

        self._sample_cpu()
        self._sample_gpu()

    def overlay_lines(self) -> list[str]:
        cpu_text = "N/A" if self._latest_cpu is None else f"{self._latest_cpu:.1f}%"
        ram_text = "N/A" if self._latest_ram_mb is None else f"{self._latest_ram_mb:.0f} MB"
        gpu_util_text = "N/A" if self._latest_gpu_util is None else f"{self._latest_gpu_util:.1f}%"
        gpu_mem_text = "N/A" if self._latest_gpu_mem_mb is None else f"{self._latest_gpu_mem_mb:.0f} MB"

        return [
            f"CPU(proc): {cpu_text}",
            f"RAM(proc): {ram_text}",
            f"GPU(util): {gpu_util_text}",
            f"GPU mem(proc): {gpu_mem_text}",
        ]

    def draw_overlay(self, frame, x: int = 12, y: int = 24) -> None:
        # Imported lazily to avoid hard dependency at module import time.
        import cv2  # type: ignore

        lines = self.overlay_lines()
        line_h = 22
        width = 260
        height = line_h * len(lines) + 14

        cv2.rectangle(frame, (x - 8, y - 20), (x - 8 + width, y - 20 + height), (0, 0, 0), -1)
        for i, txt in enumerate(lines):
            yy = y + i * line_h
            cv2.putText(frame, txt, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

    @staticmethod
    def _avg(vals: list[float]) -> float:
        return sum(vals) / len(vals) if vals else 0.0

    def print_averages(self, header: str = "Average process resource usage") -> None:
        avg_cpu = self._avg(self._cpu_samples)
        avg_gpu_util = self._avg(self._gpu_util_samples)
        avg_gpu_mem = self._avg(self._gpu_mem_samples_mb)

        print(header)
        print(f"  Avg CPU (process): {avg_cpu:.2f}%")
        if self._gpu_util_samples:
            print(f"  Avg GPU util (system): {avg_gpu_util:.2f}%")
        else:
            print("  Avg GPU util (system): N/A")
        if self._gpu_mem_samples_mb:
            print(f"  Avg GPU mem (process): {avg_gpu_mem:.2f} MB")
        else:
            print("  Avg GPU mem (process): N/A")

    def close(self) -> None:
        if self._nvml_ready and pynvml is not None:
            try:
                pynvml.nvmlShutdown()
            except Exception:
                pass
