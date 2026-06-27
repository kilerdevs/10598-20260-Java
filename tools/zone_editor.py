"""
Zone Editor
-----------
GUI tool for creating and editing field zones (deploy/zones.json).
Displays the 2026 FRC field image and lets you draw polygon zones
that trigger actions when the robot enters them.

Run from the project root:  python tools/zone_editor.py

Requirements: Python 3.8+ with tkinter (standard library).
              Optional: Pillow (pip install Pillow) for the field background image.
"""

import json
import os
import tkinter as tk
from tkinter import ttk, messagebox

# ── Paths ────────────────────────────────────────────────────────────────────
ZONES_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "src", "main", "deploy", "zones.json"
))
NAVGRID_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "src", "main", "deploy", "pathplanner", "navgrid.json"
))

# Field image from WPILib Elastic install (tried in order)
FIELD_IMAGE_PATHS = [
    "C:/Users/Public/wpilib/2026/elastic/data/flutter_assets/assets/fields/2026-field.png",
    "C:/Users/Public/wpilib/2025/elastic/data/flutter_assets/assets/fields/2025-field.png",
]
FIELD_JSON_PATH = (
    "C:/Users/Public/wpilib/2026/elastic/data/flutter_assets/assets/fields/2026-rebuilt.json"
)

# Field dimensions (meters)
FIELD_LENGTH = 16.54
FIELD_WIDTH = 8.07

# Field image corners (pixels in the source PNG) — from 2026-rebuilt.json
IMG_FIELD_TOP_LEFT = (524, 94)
IMG_FIELD_BOTTOM_RIGHT = (3378, 1490)

# ── Palette ──────────────────────────────────────────────────────────────────
BG = "#1e1e1e"
BG_PANEL = "#252526"
BG_ROW = "#2d2d2d"
FG = "#d4d4d4"
FG_DIM = "#6a6a6a"
ACCENT = "#007acc"
ACCENT_HO = "#1a8ad4"
BORDER = "#3e3e3e"
ENTRY_BG = "#3c3c3c"
ENTRY_FG = "#ffffff"

ZONE_COLORS = [
    "#4fc3f7", "#81c784", "#ffb74d", "#e57373",
    "#ba68c8", "#4db6ac", "#fff176", "#f06292",
]

KNOWN_ACTIONS = [
    "PREPARE_SHOOT",
    "AUTO_INTAKE",
    "SLOW_MODE",
    "AIM_AT_TARGET",
    "SPIN_UP_SHOOTER",
    "LED_SIGNAL",
]


# ── I/O ──────────────────────────────────────────────────────────────────────

def load_zones() -> dict:
    if not os.path.exists(ZONES_PATH):
        return {"fieldLength": FIELD_LENGTH, "fieldWidth": FIELD_WIDTH, "zones": []}
    with open(ZONES_PATH, "r") as f:
        return json.load(f)


def save_zones(data: dict):
    with open(ZONES_PATH, "w") as f:
        json.dump(data, f, indent=2)


def load_navgrid():
    if not os.path.exists(NAVGRID_PATH):
        return None
    with open(NAVGRID_PATH, "r") as f:
        return json.load(f)


def flat_button(parent, text, command, bg=BG_ROW, fg=FG, hover=None, **kw):
    hover = hover or bg
    btn = tk.Button(
        parent, text=text, command=command,
        bg=bg, fg=fg, activebackground=hover, activeforeground=fg,
        relief="flat", bd=0, cursor="hand2",
        font=("Segoe UI", 9), padx=14, pady=5, **kw,
    )
    btn.bind("<Enter>", lambda _: btn.config(bg=hover))
    btn.bind("<Leave>", lambda _: btn.config(bg=bg))
    return btn


# ── Main app ─────────────────────────────────────────────────────────────────

class ZoneEditor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Zone Editor — FRC Field Zones")
        self.configure(bg=BG)
        self.minsize(900, 600)
        self.geometry("1200x700")

        self._data = load_zones()
        self._navgrid = load_navgrid()
        self._unsaved = False

        self._canvas_w = 800
        self._canvas_h = int(self._canvas_w * FIELD_WIDTH / FIELD_LENGTH)

        # Drawing state
        self._current_vertices = []  # vertices being placed (meters)
        self._selected_zone_idx = None
        self._dragging_vertex = None  # (zone_idx, vert_idx)
        self._name_trace_id = None
        self._action_trace_id = None

        # Field background image (Pillow optional)
        self._field_photo = None
        self._load_field_image()

        self._build_ui()
        self._draw_field()

    # ── Field image ──────────────────────────────────────────────────────────

    def _load_field_image(self):
        try:
            from PIL import Image, ImageTk
        except ImportError:
            print("[ZoneEditor] Pillow not installed — using navgrid for field background")
            print("             Install with: pip install Pillow")
            return

        for path in FIELD_IMAGE_PATHS:
            if not os.path.exists(path):
                continue
            try:
                img = Image.open(path)

                # Read corner mapping from JSON if available
                tl = IMG_FIELD_TOP_LEFT
                br = IMG_FIELD_BOTTOM_RIGHT
                if os.path.exists(FIELD_JSON_PATH):
                    with open(FIELD_JSON_PATH, "r") as f:
                        fj = json.load(f)
                    fc = fj.get("field-corners", {})
                    tl = tuple(fc.get("top-left", tl))
                    br = tuple(fc.get("bottom-right", br))

                img = img.crop((tl[0], tl[1], br[0], br[1]))
                img = img.resize((self._canvas_w, self._canvas_h), Image.LANCZOS)
                self._field_photo = ImageTk.PhotoImage(img)
                print(f"[ZoneEditor] Loaded field image: {path}")
                return
            except Exception as e:
                print(f"[ZoneEditor] Failed to load {path}: {e}")

    # ── Coordinate conversion ────────────────────────────────────────────────

    def _m_to_px(self, x_m, y_m):
        px = x_m / FIELD_LENGTH * self._canvas_w
        py = (1.0 - y_m / FIELD_WIDTH) * self._canvas_h
        return px, py

    def _px_to_m(self, px, py):
        x_m = px / self._canvas_w * FIELD_LENGTH
        y_m = (1.0 - py / self._canvas_h) * FIELD_WIDTH
        return x_m, y_m

    # ── UI build ─────────────────────────────────────────────────────────────

    def _build_ui(self):
        main = tk.PanedWindow(self, orient=tk.HORIZONTAL, bg=BG, sashwidth=4, sashrelief="flat")
        main.pack(fill="both", expand=True, padx=8, pady=8)

        # ── Left: canvas ─────────────────────────────────────────────────────
        canvas_frame = tk.Frame(main, bg=BG)
        main.add(canvas_frame, stretch="always")

        self._canvas = tk.Canvas(
            canvas_frame,
            width=self._canvas_w, height=self._canvas_h,
            bg="#2a2a2a", highlightthickness=1, highlightbackground=BORDER,
        )
        self._canvas.pack(padx=4, pady=4)
        self._canvas.bind("<Button-1>", self._on_canvas_click)
        self._canvas.bind("<Button-3>", self._on_canvas_right_click)
        self._canvas.bind("<Motion>", self._on_canvas_motion)
        self._canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.bind("<Escape>", self._on_escape)
        self.bind("<Delete>", lambda _: self._delete_zone())

        coord_bar = tk.Frame(canvas_frame, bg=BG)
        coord_bar.pack(fill="x", padx=6, pady=(2, 0))
        self._coord_var = tk.StringVar(value="X: —  Y: —")
        tk.Label(coord_bar, textvariable=self._coord_var,
                 bg=BG, fg=FG_DIM, font=("Consolas", 9), anchor="w").pack(side="left")

        tk.Label(
            canvas_frame,
            text="Left-click: add vertex / select  |  Right-click: finish polygon"
                 "  |  Drag: move vertex  |  Esc: cancel",
            bg=BG, fg=FG_DIM, font=("Segoe UI", 8), anchor="w",
        ).pack(fill="x", padx=6)

        # ── Right: zone list + properties ────────────────────────────────────
        right = tk.Frame(main, bg=BG_PANEL, width=320)
        main.add(right, stretch="never")

        hdr = tk.Frame(right, bg=BG_PANEL)
        hdr.pack(fill="x", padx=8, pady=(8, 4))
        tk.Label(hdr, text="ZONES", bg=BG_PANEL, fg=FG_DIM,
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        flat_button(hdr, "+ New", self._new_zone,
                    bg=ACCENT, fg="#fff", hover=ACCENT_HO).pack(side="right")

        self._zone_listbox = tk.Listbox(
            right, bg=BG_ROW, fg=FG, selectbackground=ACCENT,
            selectforeground="#fff", relief="flat", bd=0,
            font=("Segoe UI", 10), activestyle="none", highlightthickness=0,
        )
        self._zone_listbox.pack(fill="both", expand=True, padx=8, pady=4)
        self._zone_listbox.bind("<<ListboxSelect>>", self._on_zone_select)

        # ── Properties panel ─────────────────────────────────────────────────
        props = tk.LabelFrame(
            right, text="Zone Properties", bg=BG_PANEL, fg=FG_DIM,
            font=("Segoe UI", 9, "bold"), relief="flat", bd=1,
        )
        props.pack(fill="x", padx=8, pady=(4, 8))

        tk.Label(props, text="Name:", bg=BG_PANEL, fg=FG,
                 font=("Segoe UI", 9)).grid(row=0, column=0, sticky="w", padx=6, pady=3)
        self._name_var = tk.StringVar()
        tk.Entry(
            props, textvariable=self._name_var,
            bg=ENTRY_BG, fg=ENTRY_FG, insertbackground=ENTRY_FG,
            relief="flat", font=("Segoe UI", 9),
        ).grid(row=0, column=1, sticky="ew", padx=6, pady=3)

        tk.Label(props, text="Action:", bg=BG_PANEL, fg=FG,
                 font=("Segoe UI", 9)).grid(row=1, column=0, sticky="w", padx=6, pady=3)
        self._action_var = tk.StringVar()
        ttk.Combobox(
            props, textvariable=self._action_var,
            values=KNOWN_ACTIONS, font=("Segoe UI", 9),
        ).grid(row=1, column=1, sticky="ew", padx=6, pady=3)

        self._alliance_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            props, text="Alliance-relative (mirror for red)",
            variable=self._alliance_var,
            bg=BG_PANEL, fg=FG, selectcolor=BG_ROW,
            activebackground=BG_PANEL, activeforeground=FG,
            font=("Segoe UI", 9), command=self._on_prop_change,
        ).grid(row=2, column=0, columnspan=2, sticky="w", padx=6, pady=3)

        self._vert_count_var = tk.StringVar(value="Vertices: 0")
        tk.Label(props, textvariable=self._vert_count_var, bg=BG_PANEL, fg=FG_DIM,
                 font=("Segoe UI", 8)).grid(
            row=3, column=0, columnspan=2, sticky="w", padx=6, pady=2)

        props.columnconfigure(1, weight=1)

        # Set up variable traces after widgets exist
        self._name_trace_id = self._name_var.trace_add("write", self._on_prop_change)
        self._action_trace_id = self._action_var.trace_add("write", self._on_prop_change)

        # ── Bottom buttons ───────────────────────────────────────────────────
        btn_frame = tk.Frame(right, bg=BG_PANEL)
        btn_frame.pack(fill="x", padx=8, pady=(0, 8))
        flat_button(btn_frame, "Delete Zone", self._delete_zone,
                    bg=BG_ROW, fg="#c0392b", hover="#3a2020").pack(side="left")
        flat_button(btn_frame, "Save", self._save,
                    bg=ACCENT, fg="#fff", hover=ACCENT_HO).pack(side="right")

        # ── Status bar ───────────────────────────────────────────────────────
        self._status_var = tk.StringVar()
        tk.Label(self, textvariable=self._status_var,
                 bg=BG, fg=FG_DIM, font=("Segoe UI", 8),
                 anchor="w").pack(fill="x", padx=10, pady=(0, 4))

        self._refresh_zone_list()

    # ── Drawing ──────────────────────────────────────────────────────────────

    def _draw_field(self):
        c = self._canvas
        c.delete("all")

        # Background
        if self._field_photo:
            c.create_image(0, 0, anchor="nw", image=self._field_photo)
        else:
            self._draw_navgrid()

        # Meter grid
        for x in range(int(FIELD_LENGTH) + 1):
            px, _ = self._m_to_px(x, 0)
            c.create_line(px, 0, px, self._canvas_h, fill="#404040", dash=(2, 4))
            if x % 2 == 0:
                c.create_text(px, self._canvas_h - 8, text=f"{x}m",
                              fill=FG_DIM, font=("Consolas", 7))
        for y in range(int(FIELD_WIDTH) + 1):
            _, py = self._m_to_px(0, y)
            c.create_line(0, py, self._canvas_w, py, fill="#404040", dash=(2, 4))
            if y % 2 == 0:
                c.create_text(16, py, text=f"{y}m", fill=FG_DIM, font=("Consolas", 7))

        # Zones
        for i, zone in enumerate(self._data.get("zones", [])):
            self._draw_zone(zone, i)

        # In-progress polygon
        if self._current_vertices:
            self._draw_in_progress()

        self._update_status()

    def _draw_navgrid(self):
        if not self._navgrid:
            return
        grid = self._navgrid["grid"]
        node_size = self._navgrid["nodeSizeMeters"]
        rows = len(grid)
        for r in range(rows):
            for col in range(len(grid[r])):
                if grid[r][col]:
                    x_m = col * node_size
                    y_m = (rows - 1 - r) * node_size
                    px1, py1 = self._m_to_px(x_m, y_m + node_size)
                    px2, py2 = self._m_to_px(x_m + node_size, y_m)
                    self._canvas.create_rectangle(
                        px1, py1, px2, py2, fill="#3a3a3a", outline="")

    def _draw_zone(self, zone: dict, idx: int):
        verts = zone.get("vertices", [])
        if len(verts) < 3:
            return

        color = ZONE_COLORS[idx % len(ZONE_COLORS)]
        is_selected = idx == self._selected_zone_idx

        points = []
        for v in verts:
            px, py = self._m_to_px(v["x"], v["y"])
            points.extend([px, py])

        self._canvas.create_polygon(
            points, fill=color, outline=color,
            width=3 if is_selected else 2,
            stipple="" if is_selected else "gray50",
        )

        # Label at centroid
        cx = sum(v["x"] for v in verts) / len(verts)
        cy = sum(v["y"] for v in verts) / len(verts)
        lpx, lpy = self._m_to_px(cx, cy)
        self._canvas.create_text(
            lpx, lpy, text=zone.get("name", f"Zone {idx}"),
            fill="#ffffff", font=("Segoe UI", 10, "bold"),
        )
        self._canvas.create_text(
            lpx, lpy + 14, text=f"[{zone.get('action', '')}]",
            fill="#cccccc", font=("Segoe UI", 8),
        )

        # Drag handles when selected
        if is_selected:
            for v in verts:
                vpx, vpy = self._m_to_px(v["x"], v["y"])
                self._canvas.create_oval(
                    vpx - 5, vpy - 5, vpx + 5, vpy + 5,
                    fill="#ffffff", outline=color, width=2,
                )

    def _draw_in_progress(self):
        for i, (xm, ym) in enumerate(self._current_vertices):
            x1, y1 = self._m_to_px(xm, ym)
            if i + 1 < len(self._current_vertices):
                x2, y2 = self._m_to_px(*self._current_vertices[i + 1])
                self._canvas.create_line(
                    x1, y1, x2, y2, fill="#ffff00", width=2, dash=(4, 2))
            self._canvas.create_oval(
                x1 - 4, y1 - 4, x1 + 4, y1 + 4,
                fill="#ffff00", outline="#ffffff", width=1,
            )

    # ── Canvas events ────────────────────────────────────────────────────────

    def _on_canvas_click(self, event):
        mx, my = self._px_to_m(event.x, event.y)

        # Vertex drag start?
        if self._selected_zone_idx is not None:
            zone = self._data["zones"][self._selected_zone_idx]
            for vi, v in enumerate(zone["vertices"]):
                vpx, vpy = self._m_to_px(v["x"], v["y"])
                if abs(event.x - vpx) < 8 and abs(event.y - vpy) < 8:
                    self._dragging_vertex = (self._selected_zone_idx, vi)
                    return

        # Select existing zone?
        if not self._current_vertices:
            for i, zone in enumerate(self._data.get("zones", [])):
                if self._point_in_zone(mx, my, zone):
                    self._selected_zone_idx = i
                    self._zone_listbox.selection_clear(0, tk.END)
                    self._zone_listbox.selection_set(i)
                    self._load_zone_props(i)
                    self._draw_field()
                    return

        # Place vertex
        self._current_vertices.append((mx, my))
        self._draw_field()

    def _on_canvas_right_click(self, event):
        if len(self._current_vertices) < 3:
            if self._current_vertices:
                messagebox.showinfo(
                    "Zone Editor", "Need at least 3 vertices for a polygon.")
            return

        verts = [{"x": round(x, 3), "y": round(y, 3)}
                 for x, y in self._current_vertices]
        self._data["zones"].append({
            "name": f"Zone{len(self._data['zones'])}",
            "action": "PREPARE_SHOOT",
            "allianceRelative": True,
            "vertices": verts,
        })
        self._current_vertices = []
        self._unsaved = True

        idx = len(self._data["zones"]) - 1
        self._selected_zone_idx = idx
        self._refresh_zone_list()
        self._zone_listbox.selection_set(idx)
        self._load_zone_props(idx)
        self._draw_field()

    def _on_canvas_motion(self, event):
        mx, my = self._px_to_m(event.x, event.y)
        self._coord_var.set(f"X: {mx:.2f} m  Y: {my:.2f} m")

    def _on_canvas_drag(self, event):
        if self._dragging_vertex is None:
            return
        zi, vi = self._dragging_vertex
        mx, my = self._px_to_m(event.x, event.y)
        mx = max(0.0, min(FIELD_LENGTH, mx))
        my = max(0.0, min(FIELD_WIDTH, my))
        self._data["zones"][zi]["vertices"][vi]["x"] = round(mx, 3)
        self._data["zones"][zi]["vertices"][vi]["y"] = round(my, 3)
        self._unsaved = True
        self._draw_field()

    def _on_canvas_release(self, event):
        if self._dragging_vertex is not None:
            self._dragging_vertex = None
            self._load_zone_props(self._selected_zone_idx)

    def _on_escape(self, event):
        if self._current_vertices:
            self._current_vertices = []
            self._draw_field()
            self._flash_status("Drawing cancelled")

    # ── Geometry helper ──────────────────────────────────────────────────────

    @staticmethod
    def _point_in_zone(x, y, zone):
        verts = zone.get("vertices", [])
        n = len(verts)
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = verts[i]["x"], verts[i]["y"]
            xj, yj = verts[j]["x"], verts[j]["y"]
            if ((yi > y) != (yj > y)) and \
               (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    # ── Zone list / properties ───────────────────────────────────────────────

    def _refresh_zone_list(self):
        self._zone_listbox.delete(0, tk.END)
        for i, zone in enumerate(self._data.get("zones", [])):
            name = zone.get("name", f"Zone {i}")
            action = zone.get("action", "")
            self._zone_listbox.insert(tk.END, f"  {name}  [{action}]")

    def _on_zone_select(self, event):
        sel = self._zone_listbox.curselection()
        if not sel:
            return
        self._selected_zone_idx = sel[0]
        self._load_zone_props(sel[0])
        self._draw_field()

    def _load_zone_props(self, idx):
        if idx is None or idx >= len(self._data["zones"]):
            return
        zone = self._data["zones"][idx]

        # Remove traces to avoid recursive updates
        if self._name_trace_id:
            self._name_var.trace_remove("write", self._name_trace_id)
        if self._action_trace_id:
            self._action_var.trace_remove("write", self._action_trace_id)

        self._name_var.set(zone.get("name", ""))
        self._action_var.set(zone.get("action", ""))
        self._alliance_var.set(zone.get("allianceRelative", True))
        self._vert_count_var.set(f"Vertices: {len(zone.get('vertices', []))}")

        self._name_trace_id = self._name_var.trace_add("write", self._on_prop_change)
        self._action_trace_id = self._action_var.trace_add("write", self._on_prop_change)

    def _on_prop_change(self, *_):
        if self._selected_zone_idx is None:
            return
        idx = self._selected_zone_idx
        if idx >= len(self._data["zones"]):
            return
        zone = self._data["zones"][idx]
        zone["name"] = self._name_var.get()
        zone["action"] = self._action_var.get()
        zone["allianceRelative"] = self._alliance_var.get()
        self._unsaved = True
        self._refresh_zone_list()
        self._zone_listbox.selection_set(idx)
        self._draw_field()

    # ── Actions ──────────────────────────────────────────────────────────────

    def _new_zone(self):
        self._current_vertices = []
        self._selected_zone_idx = None
        self._zone_listbox.selection_clear(0, tk.END)
        self._draw_field()
        self._flash_status(
            "Click to place vertices. Right-click to finish polygon.")

    def _delete_zone(self):
        if self._selected_zone_idx is None:
            return
        idx = self._selected_zone_idx
        name = self._data["zones"][idx].get("name", f"Zone {idx}")
        if not messagebox.askyesno("Delete Zone", f"Delete zone '{name}'?"):
            return
        del self._data["zones"][idx]
        self._selected_zone_idx = None
        self._unsaved = True
        self._refresh_zone_list()
        self._draw_field()

    def _save(self):
        if self._current_vertices:
            messagebox.showinfo(
                "Zone Editor",
                "Finish or cancel the current polygon first.\n"
                "Right-click to finish, Escape to cancel.")
            return
        save_zones(self._data)
        self._unsaved = False
        self._flash_status(
            f"Saved {len(self._data['zones'])} zones — redeploy to apply")

    def _update_status(self):
        n = len(self._data.get("zones", []))
        parts = [f"{n} zone(s) loaded"]
        if self._current_vertices:
            parts.append(f"drawing: {len(self._current_vertices)} vertices")
        if self._unsaved:
            parts.append("unsaved changes")
        self._status_var.set("  |  ".join(parts))

    def _flash_status(self, msg):
        self._status_var.set(msg)
        self.after(4000, self._update_status)


# ── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = ZoneEditor()
    app.mainloop()
