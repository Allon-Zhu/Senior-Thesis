from tkinter import Button, Entry, Frame, Label, OptionMenu, StringVar, Toplevel, LEFT, RIGHT, messagebox


class Scan2VDialog:
    def __init__(self, parent, names):
        self.top = Toplevel(parent)
        self.top.title("Scan 2V")
        self.top.transient(parent)
        self.top.grab_set()
        self.result = None

        self.v1 = StringVar(value=names[0] if names else "")
        self.v2 = StringVar(value=names[1] if len(names) > 1 else (names[0] if names else ""))
        self.a1s = StringVar(value="5.0")
        self.a1e = StringVar(value="20.0")
        self.a1n = StringVar(value="21")
        self.a2s = StringVar(value="5.0")
        self.a2e = StringVar(value="20.0")
        self.a2n = StringVar(value="21")
        self.pd1 = StringVar(value="3")
        self.pd2 = StringVar(value="4")

        frame = Frame(self.top)
        frame.pack(padx=12, pady=12)

        def add_row(row, label_text, widget):
            Label(frame, text=label_text).grid(row=row, column=0, sticky="e", padx=6, pady=4)
            widget.grid(row=row, column=1, sticky="w", padx=6, pady=4)

        add_row(0, "Voltage #1", OptionMenu(frame, self.v1, *names))
        add_row(1, "Voltage #2", OptionMenu(frame, self.v2, *names))

        axis1_frame = Frame(frame)
        Entry(axis1_frame, width=10, textvariable=self.a1s).pack(side=LEFT, padx=(0, 6))
        Entry(axis1_frame, width=10, textvariable=self.a1e).pack(side=LEFT, padx=(0, 6))
        Entry(axis1_frame, width=6, textvariable=self.a1n).pack(side=LEFT)
        add_row(2, "Axis1 start / stop / N", axis1_frame)

        axis2_frame = Frame(frame)
        Entry(axis2_frame, width=10, textvariable=self.a2s).pack(side=LEFT, padx=(0, 6))
        Entry(axis2_frame, width=10, textvariable=self.a2e).pack(side=LEFT, padx=(0, 6))
        Entry(axis2_frame, width=6, textvariable=self.a2n).pack(side=LEFT)
        add_row(3, "Axis2 start / stop / N", axis2_frame)

        pd_frame = Frame(frame)
        Entry(pd_frame, width=6, textvariable=self.pd1).pack(side=LEFT, padx=(0, 8))
        Entry(pd_frame, width=6, textvariable=self.pd2).pack(side=LEFT)
        add_row(4, "PD indices (1-4)", pd_frame)

        buttons = Frame(frame)
        buttons.grid(row=5, column=0, columnspan=2, pady=(10, 0), sticky="e")
        Button(buttons, text="Cancel", command=self.top.destroy).pack(side=RIGHT, padx=6)
        Button(buttons, text="Start", command=self._ok).pack(side=RIGHT)

    def _ok(self):
        try:
            result = (
                self.v1.get().strip(),
                self.v2.get().strip(),
                float(self.a1s.get()),
                float(self.a1e.get()),
                int(self.a1n.get()),
                float(self.a2s.get()),
                float(self.a2e.get()),
                int(self.a2n.get()),
                int(self.pd1.get()),
                int(self.pd2.get()),
            )
            if result[4] < 2 or result[7] < 2:
                raise ValueError("N must be >= 2.")
            if not (1 <= result[8] <= 4 and 1 <= result[9] <= 4):
                raise ValueError("PD indices must be in 1..4.")
            self.result = result
            self.top.destroy()
        except Exception as exc:
            messagebox.showerror("Invalid input", str(exc), parent=self.top)
