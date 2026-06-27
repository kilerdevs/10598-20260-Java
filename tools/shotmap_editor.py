"""
Shot Map Editor
---------------
GUI tool for editing deploy/shotmap.json.
Run from the project root:  python tools/shotmap_editor.py

Requirements: Python 3.8+ with tkinter (included in standard library).
"""

import json
import os
import tkinter as tk
from tkinter import ttk

# ── File path ─────────────────────────────────────────────────────────────────
SHOTMAP_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "src", "main", "deploy", "shotmap.json"
))

COLUMNS = ("distance", "rpm", "angle")
HEADERS = {"distance": "Distance (m)", "rpm": "RPM", "angle": "Angle (encoder)"}

# ── Palette ───────────────────────────────────────────────────────────────────
BG        = "#1e1e1e"
BG_PANEL  = "#252526"
BG_ROW    = "#2d2d2d"
BG_ALT    = "#262626"
FG        = "#d4d4d4"
FG_DIM    = "#6a6a6a"
ACCENT    = "#007acc"
ACCENT_HO = "#1a8ad4"
SEL_BG    = "#094771"
BORDER    = "#3e3e3e"
RED       = "#c0392b"
RED_HO    = "#e74c3c"
ENTRY_BG  = "#3c3c3c"
ENTRY_FG  = "#ffffff"


# ── I/O ───────────────────────────────────────────────────────────────────────

def load_data() -> list[dict]:
    if not os.path.exists(SHOTMAP_PATH):
        return []
    with open(SHOTMAP_PATH, "r") as f:
        return json.load(f)


def save_data(rows: list[dict]) -> list[dict]:
    rows_sorted = sorted(rows, key=lambda r: r["distance"])
    with open(SHOTMAP_PATH, "w") as f:
        json.dump(rows_sorted, f, indent=2)
    return rows_sorted


# ── Flat button helper ────────────────────────────────────────────────────────

def flat_button(parent, text, command, bg=BG_ROW, fg=FG, hover=None, width=None):
    hover = hover or bg
    kw = dict(
        text=text, command=command,
        bg=bg, fg=fg, activebackground=hover, activeforeground=fg,
        relief="flat", bd=0, cursor="hand2",
        font=("Segoe UI", 9), padx=14, pady=5,
    )
    if width:
        kw["width"] = width
    btn = tk.Button(parent, **kw)
    btn.bind("<Enter>", lambda _: btn.config(bg=hover))
    btn.bind("<Leave>", lambda _: btn.config(bg=bg))
    return btn


# ── Main window ───────────────────────────────────────────────────────────────

class ShotMapEditor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Shot Map Editor")
        self.configure(bg=BG)
        self.minsize(540, 360)
        self.geometry("600x420")

        self._rows: list[dict] = load_data()
        self._unsaved = False

        # inline-edit state
        self._edit_entry: tk.Entry | None = None
        self._edit_iid:   str | None = None
        self._edit_col:   int | None = None

        self._apply_styles()
        self._build_ui()
        self._refresh_table()

    # ── Styles ────────────────────────────────────────────────────────────────

    def _apply_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        style.configure("Treeview",
            background=BG_ROW, foreground=FG,
            fieldbackground=BG_ROW, borderwidth=0,
            rowheight=28, font=("Segoe UI", 9),
        )
        style.configure("Treeview.Heading",
            background=BG_PANEL, foreground=FG_DIM,
            borderwidth=0, relief="flat",
            font=("Segoe UI", 8, "bold"),
        )
        style.map("Treeview",
            background=[("selected", SEL_BG)],
            foreground=[("selected", "#ffffff")],
        )
        style.map("Treeview.Heading",
            background=[("active", BG_PANEL)],
        )
        style.configure("Vertical.TScrollbar",
            background=BG_PANEL, troughcolor=BG,
            arrowcolor=FG_DIM, borderwidth=0, relief="flat",
        )
        style.map("Vertical.TScrollbar",
            background=[("active", BORDER)],
        )

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Table ─────────────────────────────────────────────────────────────
        tree_frame = tk.Frame(self, bg=BG)
        tree_frame.pack(fill="both", expand=True, padx=16, pady=(14, 0))

        self._tree = ttk.Treeview(tree_frame, columns=COLUMNS, show="headings", selectmode="browse")

        col_widths = {"distance": 150, "rpm": 130, "angle": 170}
        for col in COLUMNS:
            self._tree.heading(col, text=HEADERS[col])
            self._tree.column(col, width=col_widths[col], anchor="center", minwidth=80)

        vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self._tree.yview, style="Vertical.TScrollbar")
        self._tree.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        self._tree.pack(side="left", fill="both", expand=True)

        # alternating row tags
        self._tree.tag_configure("odd",  background=BG_ALT)
        self._tree.tag_configure("even", background=BG_ROW)

        self._tree.bind("<Button-1>",   self._on_click)
        self._tree.bind("<Delete>",     lambda _: self._delete_selected())
        self._tree.bind("<Escape>",     lambda _: self._cancel_edit())

        # ── Bottom bar ────────────────────────────────────────────────────────
        bar = tk.Frame(self, bg=BG, pady=10)
        bar.pack(fill="x", padx=16)

        flat_button(bar, "+ Add row", self._add_row,
                    bg=BG_ROW, hover=BORDER).pack(side="left", padx=(0, 6))
        flat_button(bar, "Delete", self._delete_selected,
                    bg=BG_ROW, fg="#c0392b", hover=BG_ROW).pack(side="left")

        flat_button(bar, "Save", self._save,
                    bg=ACCENT, fg="#ffffff", hover=ACCENT_HO).pack(side="right")

        # ── Status ────────────────────────────────────────────────────────────
        self._status_var = tk.StringVar()
        tk.Label(self, textvariable=self._status_var,
                 bg=BG, fg=FG_DIM, font=("Segoe UI", 8),
                 anchor="w").pack(fill="x", padx=16, pady=(0, 8))

    # ── Table refresh ─────────────────────────────────────────────────────────

    def _refresh_table(self, keep_selection: str | None = None):
        self._cancel_edit()
        self._rows.sort(key=lambda r: r["distance"])

        for item in self._tree.get_children():
            self._tree.delete(item)

        for i, row in enumerate(self._rows):
            tag = "even" if i % 2 == 0 else "odd"
            self._tree.insert("", "end", values=(
                f"{row['distance']:.2f}",
                f"{row['rpm']:.0f}",
                f"{row['angle']:.2f}",
            ), tags=(tag,))

        if keep_selection:
            children = self._tree.get_children()
            if children:
                target = min(int(keep_selection), len(children) - 1)
                iid = children[target]
                self._tree.selection_set(iid)
                self._tree.see(iid)

        self._update_status()

    def _update_status(self):
        n = len(self._rows)
        suffix = "  •  unsaved changes" if self._unsaved else ""
        if n == 0:
            self._status_var.set("No entries" + suffix)
        elif n == 1:
            self._status_var.set("1 entry — need ≥2 for interpolation" + suffix)
        else:
            lo = min(r["distance"] for r in self._rows)
            hi = max(r["distance"] for r in self._rows)
            self._status_var.set(f"{n} entries  ·  {lo:.2f} m – {hi:.2f} m" + suffix)

    # ── Inline cell editing ───────────────────────────────────────────────────

    def _on_click(self, event):
        region = self._tree.identify_region(event.x, event.y)
        if region != "cell":
            self._cancel_edit()
            return

        iid = self._tree.identify_row(event.y)
        col_id = self._tree.identify_column(event.x)   # "#1", "#2", "#3"
        col_idx = int(col_id[1:]) - 1

        # clicking same cell → ignore (already editing)
        if iid == self._edit_iid and col_idx == self._edit_col:
            return

        self._commit_edit()
        self._tree.selection_set(iid)
        self._start_edit(iid, col_idx)

    def _start_edit(self, iid: str, col_idx: int):
        bbox = self._tree.bbox(iid, COLUMNS[col_idx])
        if not bbox:
            return
        x, y, w, h = bbox

        current_val = self._tree.item(iid, "values")[col_idx]

        self._edit_iid  = iid
        self._edit_col  = col_idx

        entry = tk.Entry(
            self._tree,
            bg=ENTRY_BG, fg=ENTRY_FG, insertbackground=ENTRY_FG,
            selectbackground=ACCENT, selectforeground="#ffffff",
            relief="flat", bd=0, highlightthickness=1,
            highlightbackground=ACCENT, highlightcolor=ACCENT,
            font=("Segoe UI", 9), justify="center",
        )
        entry.place(x=x, y=y, width=w, height=h)
        entry.insert(0, current_val)
        entry.select_range(0, "end")
        entry.focus_set()

        entry.bind("<Return>",   lambda _: self._commit_edit(advance=True))
        entry.bind("<Tab>",      lambda _: self._commit_edit(advance=True))
        entry.bind("<Escape>",   lambda _: self._cancel_edit())
        entry.bind("<FocusOut>", lambda _: self._commit_edit())

        self._edit_entry = entry

    def _commit_edit(self, advance=False):
        if self._edit_entry is None:
            return

        raw = self._edit_entry.get().strip()
        iid     = self._edit_iid
        col_idx = self._edit_col

        self._edit_entry.destroy()
        self._edit_entry = None
        self._edit_iid   = None
        self._edit_col   = None

        try:
            value = float(raw)
        except ValueError:
            return  # discard invalid input silently

        # find row index by iid
        children = list(self._tree.get_children())
        try:
            row_idx = children.index(iid)
        except ValueError:
            return

        key = COLUMNS[col_idx]
        self._rows[row_idx][key] = value
        self._unsaved = True

        # update the single cell without full rebuild
        vals = list(self._tree.item(iid, "values"))
        vals[col_idx] = f"{value:.2f}" if key != "rpm" else f"{value:.0f}"
        self._tree.item(iid, values=vals)
        self._update_status()

        if advance:
            next_col = (col_idx + 1) % len(COLUMNS)
            next_iid = iid
            if next_col == 0:
                # wrap to next row
                idx = children.index(iid)
                if idx + 1 < len(children):
                    next_iid = children[idx + 1]
                else:
                    return
            self._tree.selection_set(next_iid)
            self.after(10, lambda: self._start_edit(next_iid, next_col))

    def _cancel_edit(self):
        if self._edit_entry is not None:
            self._edit_entry.destroy()
            self._edit_entry = None
            self._edit_iid   = None
            self._edit_col   = None

    # ── Actions ───────────────────────────────────────────────────────────────

    def _add_row(self):
        self._commit_edit()
        distances = {r["distance"] for r in self._rows}
        # suggest next round distance
        candidate = 1.0
        while candidate in distances:
            candidate = round(candidate + 0.5, 1)
        self._rows.append({"distance": candidate, "rpm": 3000.0, "angle": 25.0})
        self._unsaved = True
        self._rows.sort(key=lambda r: r["distance"])
        self._refresh_table()
        # start editing the distance cell of the new row
        children = self._tree.get_children()
        idx = next((i for i, r in enumerate(self._rows) if r["distance"] == candidate), None)
        if idx is not None and idx < len(children):
            iid = children[idx]
            self._tree.selection_set(iid)
            self._tree.see(iid)
            self.after(50, lambda: self._start_edit(iid, 0))

    def _delete_selected(self):
        self._cancel_edit()
        sel = self._tree.selection()
        if not sel:
            return
        children = list(self._tree.get_children())
        row_idx = children.index(sel[0])
        del self._rows[row_idx]
        self._unsaved = True
        self._refresh_table(keep_selection=str(row_idx))

    def _save(self):
        self._commit_edit()
        if len(self._rows) < 2:
            self._flash_status("Need at least 2 entries to save.")
            return
        self._rows = save_data(self._rows)
        self._unsaved = False
        self._refresh_table()
        self._flash_status(f"Saved {len(self._rows)} entries  ·  redeploy to apply")

    def _flash_status(self, msg: str):
        self._status_var.set(msg)
        self.after(3000, self._update_status)


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = ShotMapEditor()
    app.mainloop()
