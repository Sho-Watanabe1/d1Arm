import json
import math
import os
import select
import subprocess
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox


JOINT_LIMITS = {
    0: (-135, 135),
    1: (-90, 90),
    2: (-90, 90),
    3: (-135, 135),
    4: (-90, 90),
    5: (-135, 135),
    6: (-27, 69),
}

ARM_DH = [
    {"d": 0.1316, "a": 0.0, "alpha": math.pi / 2.0, "offset": 0.0},
    {"d": 0.0, "a": 0.27, "alpha": 0.0, "offset": math.pi / 2.0},
    {"d": 0.0, "a": 0.041325, "alpha": math.pi / 2.0, "offset": 0.0},
    {"d": 0.204675, "a": 0.0, "alpha": math.pi / 2.0, "offset": 0.0},
    {"d": 0.0, "a": 0.0, "alpha": -math.pi / 2.0, "offset": 0.0},
    {"d": 0.205744, "a": 0.0, "alpha": 0.0, "offset": 0.0},
]


def dh_transform(theta, d, a, alpha):
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matrix_multiply(a, b):
    result = [[0.0 for _ in range(4)] for _ in range(4)]
    for i in range(4):
        for j in range(4):
            result[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return result


def arm_joint_positions(angles):
    transform = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    positions = [(0.0, 0.0, 0.0)]
    for angle_deg, dh in zip(angles[:6], ARM_DH):
        theta = math.radians(float(angle_deg)) + dh["offset"]
        link_transform = dh_transform(theta, dh["d"], dh["a"], dh["alpha"])
        transform = matrix_multiply(transform, link_transform)
        positions.append((transform[0][3], transform[1][3], transform[2][3]))

    return positions


def discover_examples(project_root: Path):
    build_dir = project_root / "build"
    examples = []
    for binary in sorted(build_dir.glob("*")):
        if binary.is_file() and os.access(binary, os.X_OK):
            name = binary.name
            if name.startswith("."):
                continue
            examples.append({"name": name, "path": str(binary)})
    return examples


def build_joint_payload(angles):
    return json.dumps({
        "seq": 4,
        "address": 1,
        "funcode": 2,
        "data": {
            "mode": 1,
            "angle0": angles[0],
            "angle1": angles[1],
            "angle2": angles[2],
            "angle3": angles[3],
            "angle4": angles[4],
            "angle5": angles[5],
            "angle6": angles[6],
        },
    })


def build_joint_stream_line(angles):
    return " ".join(str(int(angle)) for angle in angles) + "\n"


def read_current_joint_angles(project_root: Path):
    executable_path = project_root / "build" / "get_arm_joint_angle"
    if not executable_path.exists():
        return None

    process = subprocess.Popen(
        [str(executable_path)],
        cwd=str(project_root),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break

        if process.stdout is None:
            break

        ready, _, _ = select.select([process.stdout], [], [], 0.1)
        if process.stdout not in ready:
            continue

        line = process.stdout.readline()
        if not line:
            continue

        try:
            process.kill()
            process.wait(timeout=0.5)
        except Exception:
            pass

        parsed_angles = []
        for token in line.strip().split(","):
            token = token.strip()
            if "servo" not in token or "_data:" not in token:
                continue
            try:
                parsed_angles.append(float(token.split(":", 1)[1].strip()))
            except ValueError:
                continue

        if len(parsed_angles) >= 7:
            return parsed_angles[:7]

        return None

    try:
        process.kill()
        process.wait(timeout=0.5)
    except Exception:
        pass

    return None


class ExampleFrontend(tk.Tk):
    def __init__(self, project_root: Path):
        super().__init__()
        self.project_root = project_root
        self.title("d1Arm Example Launcher")
        self.geometry("700x520")

        self.label = tk.Label(self, text="Choose an example to run", font=("Arial", 12))
        self.label.pack(pady=(16, 8))

        self.listbox = tk.Listbox(self, height=6, width=40)
        self.listbox.pack(padx=16, pady=8, fill=tk.BOTH, expand=True)

        button_frame = tk.Frame(self)
        button_frame.pack(pady=(0, 12))

        self.run_button = tk.Button(button_frame, text="Run Example", command=self.run_selected)
        self.run_button.pack(side=tk.LEFT, padx=6)

        self.refresh_button = tk.Button(button_frame, text="Refresh", command=self.refresh_examples)
        self.refresh_button.pack(side=tk.LEFT, padx=6)

        self.slider_frame = tk.Frame(self)
        self.slider_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=8)

        self.visualizer_frame = tk.Frame(self)
        self.visualizer_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=(0, 12))

        self.visualizer_label = tk.Label(self.visualizer_frame, text="Arm debugger", font=("Arial", 11, "bold"))
        self.visualizer_label.pack(anchor=tk.W)

        self.visualizer_canvas = tk.Canvas(self.visualizer_frame, width=700, height=340, bg="#fefefe", highlightthickness=1)
        self.visualizer_canvas.pack(fill=tk.BOTH, expand=True, pady=(4, 0))

        self.sliders = []
        self.values = []
        self.stream_process = None
        initial_angles = read_current_joint_angles(project_root)
        self.create_sliders(initial_angles)
        self.draw_arm_debug()

        self.refresh_examples()

    def create_sliders(self, initial_angles=None):
        for index, (joint_name, (low, high)) in enumerate([('J0', JOINT_LIMITS[0]), ('J1', JOINT_LIMITS[1]), ('J2', JOINT_LIMITS[2]), ('J3', JOINT_LIMITS[3]), ('J4', JOINT_LIMITS[4]), ('J5', JOINT_LIMITS[5]), ('J6', JOINT_LIMITS[6])]):
            row = tk.Frame(self.slider_frame)
            row.pack(fill=tk.X, pady=2)

            label = tk.Label(row, text=f"{joint_name} ({low}° to {high}°)", width=16, anchor=tk.W)
            label.pack(side=tk.LEFT)

            slider = tk.Scale(row, from_=low, to=high, orient=tk.HORIZONTAL, length=280)
            start_angle = 0
            if initial_angles is not None and index < len(initial_angles):
                start_angle = int(round(initial_angles[index]))
            slider.set(start_angle)
            slider.pack(side=tk.LEFT, padx=8)

            value_label = tk.Label(row, text=f"{start_angle}°", width=8)
            value_label.pack(side=tk.LEFT)

            slider.configure(command=lambda value, label=value_label, slider_index=index: self.on_slider_change(slider_index, value, label))
            self.sliders.append((slider, value_label))
            self.values.append(start_angle)

    def draw_arm_debug(self):
        canvas = self.visualizer_canvas
        if canvas is None:
            return

        canvas.delete("arm")

        width = canvas.winfo_width() or 700
        height = canvas.winfo_height() or 340
        center_x = width / 2.0
        center_y = height / 2.0
        scale = 170.0

        positions = arm_joint_positions(self.values)
        projected = []
        for x, _, z in positions:
            projected.append((center_x + x * scale, center_y - z * scale))

        canvas.create_line(center_x - 180, center_y + 110, center_x + 180, center_y + 110, fill="#888888", width=1, tags="arm")
        canvas.create_text(center_x - 180, center_y + 125, text="base", anchor=tk.NW, fill="#666666", tags="arm")

        for idx, (x1, y1) in enumerate(projected[:-1]):
            x2, y2 = projected[idx + 1]
            canvas.create_line(x1, y1, x2, y2, fill="#2563eb", width=4, tags="arm")

        for idx, (x, y) in enumerate(projected):
            fill = "#ef4444" if idx == 0 else "#1d4ed8" if idx < len(projected) - 1 else "#16a34a"
            canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=fill, outline="#111111", width=1, tags="arm")
            if idx < len(projected) - 1:
                canvas.create_text(x + 8, y - 8, text=f"J{idx}", anchor=tk.W, fill="#111111", tags="arm")

        canvas.create_text(center_x + 14, center_y - 120, text="x-z projection", anchor=tk.W, fill="#4b5563", tags="arm")

    def on_slider_change(self, slider_index, value, label):
        label.config(text=f"{float(value):.0f}°")
        self.values[slider_index] = int(float(value))
        self.draw_arm_debug()

        selection = self.listbox.curselection()
        if not selection:
            return

        example_name = self.listbox.get(selection[0])
        if example_name != "multi_joint_slider_control":
            return

        if self.stream_process is None:
            self.start_stream()

        self.send_current_angles()

    def refresh_examples(self):
        self.listbox.delete(0, tk.END)
        for example in discover_examples(self.project_root):
            self.listbox.insert(tk.END, example["name"])

    def start_stream(self):
        example = next((item for item in discover_examples(self.project_root) if item["name"] == "multi_joint_slider_control"), None)
        if not example:
            self.stream_process = None
            return

        self.stream_process = subprocess.Popen(
            [example["path"]],
            cwd=str(self.project_root),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

    def send_current_angles(self):
        if self.stream_process is None or self.stream_process.poll() is not None:
            self.start_stream()
            if self.stream_process is None:
                return

        angles = [int(value) for value in self.values]
        line = build_joint_stream_line(angles)
        if self.stream_process.stdin is not None:
            self.stream_process.stdin.write(line)
            self.stream_process.stdin.flush()

    def build_example(self, example_name: str):
        try:
            subprocess.run(["cmake", "-S", ".", "-B", "build"], cwd=str(self.project_root), check=True, capture_output=True, text=True)
            subprocess.run(["cmake", "--build", "build", "--target", example_name, "-j2"], cwd=str(self.project_root), check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as exc:
            output = exc.stderr.strip() or exc.stdout.strip() or str(exc)
            raise RuntimeError(output) from exc

    def run_selected(self):
        selection = self.listbox.curselection()
        if not selection:
            messagebox.showwarning("No selection", "Please select an example first.")
            return

        example_name = self.listbox.get(selection[0])
        examples = discover_examples(self.project_root)
        example = next((item for item in examples if item["name"] == example_name), None)

        try:
            if not example:
                self.build_example(example_name)
                examples = discover_examples(self.project_root)
                example = next((item for item in examples if item["name"] == example_name), None)

            if not example:
                messagebox.showerror("Not found", f"Could not find executable for {example_name}.")
                return

            if example_name == "multi_joint_slider_control":
                angles = [int(slider.get()) for slider, _ in self.sliders]
                self.values = angles
                payload = build_joint_payload(angles)
                self.start_stream()
                self.send_current_angles()
                messagebox.showinfo("Streaming", f"Streaming angles:\n{payload}")
            else:
                subprocess.Popen([example["path"]], cwd=str(self.project_root))
                messagebox.showinfo("Started", f"Started {example_name}")
        except OSError as exc:
            messagebox.showerror("Launch failed", f"Could not start example: {exc}")
        except RuntimeError as exc:
            messagebox.showerror("Build failed", f"Could not build {example_name}:\n{exc}")


if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent
    app = ExampleFrontend(project_root)
    app.mainloop()
