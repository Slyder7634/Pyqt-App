#!/usr/bin/env python3
"""Patches main_window.ui in place.

Edits the existing file rather than regenerating it, so anything you changed
in Qt Designer survives. Safe to run twice — each step is skipped if already
applied.

  1. Removes the TF preview card (bottom left) — replaced by ws_sim.
  2. Adds a "Keyboard Pendant" button to the header.
"""

import re
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else "main_window.ui"

with open(PATH, "r", encoding="utf-8") as fh:
    ui = fh.read()

changed = []

# ── 1. drop the TF preview card ─────────────────────────────────────────
if "tfPreviewCard" in ui:
    start = ui.index('<widget class="QFrame" name="tfPreviewCard">')
    # walk back to the <item> that wraps it
    item_start = ui.rindex("<item>", 0, start)
    # walk forward to the matching </item> by depth-counting <item> tags
    depth = 0
    i = item_start
    while True:
        nxt_open = ui.find("<item>", i + 1)
        nxt_close = ui.find("</item>", i + 1)
        if nxt_close == -1:
            raise SystemExit("malformed .ui: unbalanced <item>")
        if nxt_open != -1 and nxt_open < nxt_close:
            depth += 1
            i = nxt_open
        else:
            if depth == 0:
                item_end = nxt_close + len("</item>")
                break
            depth -= 1
            i = nxt_close

    ui = ui[:item_start] + ui[item_end:]
    # tidy the blank line left behind
    ui = re.sub(r"\n[ \t]*\n([ \t]*</layout>)", r"\n\1", ui, count=1)
    changed.append("removed tfPreviewCard")

# ── 2. add the Keyboard Pendant button ──────────────────────────────────
if 'name="keyboardPendantButton"' not in ui:
    anchor = '''          <item>
           <widget class="QPushButton" name="nexSimButton">'''
    if anchor not in ui:
        raise SystemExit("could not find nexSimButton to anchor the new button")

    button = '''          <item>
           <widget class="QPushButton" name="keyboardPendantButton">
            <property name="text"><string>Keyboard Pendant</string></property>
            <property name="class" stdset="0"><string notr="true">btnGrey</string></property>
            <property name="checkable"><bool>true</bool></property>
            <property name="minimumSize">
             <size>
              <width>0</width>
              <height>48</height>
             </size>
            </property>
           </widget>
          </item>
'''
    ui = ui.replace(anchor, button + anchor, 1)
    changed.append("added keyboardPendantButton")

# ── 3. keep the editor card at its natural height ───────────────────────
# Without the TF card below it, the editor column had nothing to absorb the
# leftover space and the NATURE box stretched to fill it.
if 'name="editorColumnSpacer"' not in ui:
    anchor = '<layout class="QVBoxLayout" name="editorColumnLayout">'
    if anchor in ui:
        # Find the </layout> that closes THIS layout, not the first nested one.
        i = ui.index(anchor)
        depth = 0
        while True:
            nxt_open = ui.find("<layout ", i + 1)
            nxt_close = ui.find("</layout>", i + 1)
            if nxt_close == -1:
                raise SystemExit("malformed .ui: unbalanced <layout>")
            if nxt_open != -1 and nxt_open < nxt_close:
                depth += 1
                i = nxt_open
            else:
                if depth == 0:
                    close = nxt_close
                    break
                depth -= 1
                i = nxt_close

        spacer = """           <item>
            <spacer name="editorColumnSpacer">
             <property name="orientation"><enum>Qt::Vertical</enum></property>
             <property name="sizeHint" stdset="0"><size><width>20</width><height>40</height></size></property>
            </spacer>
           </item>
"""
        ui = ui[:close] + spacer + ui[close:]
        changed.append("added editorColumnSpacer")

with open(PATH, "w", encoding="utf-8") as fh:
    fh.write(ui)

print("patched:", ", ".join(changed) if changed else "nothing to do")
