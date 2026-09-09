#!/usr/bin/env python3
"""Generates main_window.ui — the Point Planning screen, laid out for touch."""

W = []
def w(s=""): W.append(s)

def prop(name, kind, val, indent):
    p = " " * indent
    return f'{p}<property name="{name}">\n{p} <{kind}>{val}</{kind}>\n{p}</property>'

def size(nm, wd, ht, indent):
    p = " " * indent
    return (f'{p}<property name="{nm}">\n{p} <size>\n{p}  <width>{wd}</width>\n'
            f'{p}  <height>{ht}</height>\n{p} </size>\n{p}</property>')

def spacer(indent, orient="Vertical", wd=20, ht=40):
    p = " " * indent
    return (f'{p}<item>\n'
            f'{p} <spacer name="spacer_{len(W)}">\n'
            f'{p}  <property name="orientation"><enum>Qt::{orient}</enum></property>\n'
            f'{p}  <property name="sizeHint" stdset="0"><size><width>{wd}</width>'
            f'<height>{ht}</height></size></property>\n'
            f'{p} </spacer>\n{p}</item>')

def led_block(title, prefix, indent):
    """OPERATIONAL / FAULT / HOMING / EMERGENCY status: header + 6 LEDs."""
    p = " " * indent
    o = [f'{p}<widget class="QWidget" name="{prefix}Block" native="true">',
         f'{p} <layout class="QVBoxLayout" name="{prefix}BlockLayout">',
         f'{p}  <property name="spacing"><number>5</number></property>',
         f'{p}  <property name="leftMargin"><number>0</number></property>',
         f'{p}  <property name="rightMargin"><number>0</number></property>',
         f'{p}  <property name="topMargin"><number>0</number></property>',
         f'{p}  <property name="bottomMargin"><number>0</number></property>',
         f'{p}  <item>',
         f'{p}   <widget class="QLabel" name="{prefix}Header">',
         f'{p}    <property name="text"><string>{title}</string></property>',
         f'{p}    <property name="class" stdset="0"><string notr="true">statusHeader</string></property>',
         f'{p}   </widget>',
         f'{p}  </item>',
         f'{p}  <item>',
         f'{p}   <widget class="QFrame" name="{prefix}LedRow">',
         f'{p}    <layout class="QHBoxLayout" name="{prefix}LedRowLayout">',
         f'{p}     <property name="spacing"><number>4</number></property>',
         f'{p}     <property name="leftMargin"><number>4</number></property>',
         f'{p}     <property name="rightMargin"><number>4</number></property>',
         f'{p}     <property name="topMargin"><number>4</number></property>',
         f'{p}     <property name="bottomMargin"><number>4</number></property>']
    for j in range(1, 7):
        o += [f'{p}     <item>',
              f'{p}      <widget class="QLabel" name="{prefix}Led{j}">',
              f'{p}       <property name="text"><string/></property>',
              f'{p}       <property name="class" stdset="0"><string notr="true">led</string></property>',
              size("minimumSize", 0, 22, indent + 7),
              f'{p}      </widget>',
              f'{p}     </item>']
    o += [f'{p}    </layout>',
          f'{p}   </widget>',
          f'{p}  </item>',
          f'{p} </layout>',
          f'{p}</widget>']
    return "\n".join(o)

def jog_row(label, unit, prefix, idx, indent):
    """A touch jog row:  [J1]  (+)  [ 0.00 ]  (-)"""
    p = " " * indent
    return "\n".join([
        f'{p}<item>',
        f'{p} <layout class="QHBoxLayout" name="{prefix}Row{idx}Layout">',
        f'{p}  <property name="spacing"><number>8</number></property>',
        f'{p}  <item>',
        f'{p}   <widget class="QLabel" name="{prefix}Label{idx}">',
        f'{p}    <property name="text"><string>{label}</string></property>',
        f'{p}    <property name="class" stdset="0"><string notr="true">axisLabel</string></property>',
        size("minimumSize", 34, 0, indent + 4),
        f'{p}   </widget>',
        f'{p}  </item>',
        f'{p}  <item>',
        f'{p}   <widget class="QPushButton" name="{prefix}Plus{idx}">',
        f'{p}    <property name="text"><string>+</string></property>',
        f'{p}    <property name="class" stdset="0"><string notr="true">jogPlus</string></property>',
        f'{p}    <property name="autoRepeat"><bool>false</bool></property>',
        size("minimumSize", 62, 54, indent + 4),
        f'{p}   </widget>',
        f'{p}  </item>',
        f'{p}  <item>',
        f'{p}   <widget class="QLabel" name="{prefix}Value{idx}">',
        f'{p}    <property name="text"><string>0.00{unit}</string></property>',
        f'{p}    <property name="class" stdset="0"><string notr="true">jogValue</string></property>',
        f'{p}    <property name="alignment"><set>Qt::AlignCenter</set></property>',
        size("minimumSize", 84, 40, indent + 4),
        f'{p}   </widget>',
        f'{p}  </item>',
        f'{p}  <item>',
        f'{p}   <widget class="QPushButton" name="{prefix}Minus{idx}">',
        f'{p}    <property name="text"><string>-</string></property>',
        f'{p}    <property name="class" stdset="0"><string notr="true">jogMinus</string></property>',
        size("minimumSize", 62, 54, indent + 4),
        f'{p}   </widget>',
        f'{p}  </item>',
        f'{p} </layout>',
        f'{p}</item>',
    ])

def nav_button(text, name, checked, indent):
    p = " " * indent
    return "\n".join([
        f'{p}<item>',
        f'{p} <widget class="QPushButton" name="{name}">',
        f'{p}  <property name="text"><string>{text}</string></property>',
        f'{p}  <property name="class" stdset="0"><string notr="true">navButton</string></property>',
        f'{p}  <property name="checkable"><bool>true</bool></property>',
        f'{p}  <property name="checked"><bool>{"true" if checked else "false"}</bool></property>',
        size("minimumSize", 0, 52, indent + 2),
        f'{p} </widget>',
        f'{p}</item>',
    ])

def seg_button(text, name, checked, group, indent):
    p = " " * indent
    return "\n".join([
        f'{p}<item>',
        f'{p} <widget class="QPushButton" name="{name}">',
        f'{p}  <property name="text"><string>{text}</string></property>',
        f'{p}  <property name="class" stdset="0"><string notr="true">segButton</string></property>',
        f'{p}  <property name="checkable"><bool>true</bool></property>',
        f'{p}  <property name="checked"><bool>{"true" if checked else "false"}</bool></property>',
        size("minimumSize", 0, 46, indent + 2),
        f'{p}  <attribute name="buttonGroup"><string notr="true">{group}</string></attribute>',
        f'{p} </widget>',
        f'{p}</item>',
    ])

# ─────────────────────────────────────────────────────────────── document ────
w('<?xml version="1.0" encoding="UTF-8"?>')
w('<ui version="4.0">')
w(' <class>MainWindow</class>')
w(' <widget class="QMainWindow" name="MainWindow">')
w('  <property name="geometry"><rect><x>0</x><y>0</y><width>1920</width><height>1080</height></rect></property>')
w('  <property name="windowTitle"><string>NextUp Robot Control</string></property>')
w('  <widget class="QWidget" name="centralwidget">')
w('   <layout class="QHBoxLayout" name="rootLayout">')
w('    <property name="spacing"><number>0</number></property>')
w('    <property name="leftMargin"><number>0</number></property>')
w('    <property name="rightMargin"><number>0</number></property>')
w('    <property name="topMargin"><number>0</number></property>')
w('    <property name="bottomMargin"><number>0</number></property>')

# ───────────────────────────────────────────────────────────────── sidebar ───
w('    <item>')
w('     <widget class="QWidget" name="sidebar" native="true">')
w(size("minimumSize", 250, 0, 6))
w(size("maximumSize", 250, 16777215, 6))
w('      <layout class="QVBoxLayout" name="sidebarLayout">')
w('       <property name="spacing"><number>6</number></property>')
w('       <property name="leftMargin"><number>10</number></property>')
w('       <property name="rightMargin"><number>10</number></property>')
w('       <property name="topMargin"><number>14</number></property>')
w('       <property name="bottomMargin"><number>12</number></property>')

w('       <item>')
w('        <widget class="QLabel" name="projectManagerTitle">')
w('         <property name="text"><string>Project Manager</string></property>')
w('         <property name="class" stdset="0"><string notr="true">pmTitle</string></property>')
w('         <property name="alignment"><set>Qt::AlignCenter</set></property>')
w('        </widget>')
w('       </item>')
w('       <item>')
w('        <widget class="QLabel" name="activeProjectName">')
w('         <property name="text"><string>displayProject</string></property>')
w('         <property name="class" stdset="0"><string notr="true">pmSubtitle</string></property>')
w('         <property name="alignment"><set>Qt::AlignCenter</set></property>')
w('        </widget>')
w('       </item>')

for text, name, chk in [("Point Planning", "navPointPlanning", True),
                        ("Path Planning", "navPathPlanning", False),
                        ("IO Control", "navIoControl", False),
                        ("Main Tree", "navMainTree", False),
                        ("Client Control", "navClientControl", False),
                        ("Error Handling", "navErrorHandling", False)]:
    w(nav_button(text, name, chk, 7))

w(spacer(7, "Vertical", 20, 12))

for title, prefix in [("OPERATIONAL STATUS", "op"), ("FAULT STATUS", "fault"),
                      ("HOMING STATUS", "homing"), ("EMERGENCY STATUS", "emergency")]:
    w('       <item>')
    w(led_block(title, prefix, 8))
    w('       </item>')

w(spacer(7, "Vertical", 20, 10))

w('       <item>')
w('        <layout class="QHBoxLayout" name="homeResetLayout">')
w('         <property name="spacing"><number>8</number></property>')
for text, name in [("HOME", "homeButton"), ("RESET", "resetButton")]:
    w('         <item>')
    w(f'          <widget class="QPushButton" name="{name}">')
    w(f'           <property name="text"><string>{text}</string></property>')
    w(f'           <property name="class" stdset="0"><string notr="true">{name}</string></property>')
    w(size("minimumSize", 0, 52, 11))
    w('          </widget>')
    w('         </item>')
w('        </layout>')
w('       </item>')

for text, name in [("MOTION RESET", "motionResetButton"),
                   ("EMERGENCY\nSTOP", "emergencyStopButton")]:
    w('       <item>')
    w(f'        <widget class="QPushButton" name="{name}">')
    w(f'         <property name="text"><string>{text}</string></property>')
    w(f'         <property name="class" stdset="0"><string notr="true">{name}</string></property>')
    w(size("minimumSize", 0, 56 if "MOTION" in text else 74, 9))
    w('        </widget>')
    w('       </item>')

w('       <item>')
w('        <widget class="QLabel" name="systemStatusLabel">')
w('         <property name="text"><string>TIME : --:--:--\nUpTime: --\nLoad: -- -- --</string></property>')
w('         <property name="class" stdset="0"><string notr="true">sysStatus</string></property>')
w('        </widget>')
w('       </item>')
w('       <item>')
w('        <widget class="QPushButton" name="powerMenuButton">')
w('         <property name="text"><string>Power Menu</string></property>')
w('         <property name="class" stdset="0"><string notr="true">powerMenuButton</string></property>')
w(size("minimumSize", 0, 48, 9))
w('        </widget>')
w('       </item>')
w('      </layout>')
w('     </widget>')
w('    </item>')

# ───────────────────────────────────────────────────────────── main column ───
w('    <item>')
w('     <widget class="QWidget" name="mainArea" native="true">')
w('      <layout class="QVBoxLayout" name="mainAreaLayout">')
w('       <property name="spacing"><number>14</number></property>')
w('       <property name="leftMargin"><number>18</number></property>')
w('       <property name="rightMargin"><number>18</number></property>')
w('       <property name="topMargin"><number>16</number></property>')
w('       <property name="bottomMargin"><number>16</number></property>')

# header bar
w('       <item>')
w('        <widget class="QFrame" name="headerBar">')
w('         <layout class="QHBoxLayout" name="headerBarLayout">')
w('          <property name="spacing"><number>10</number></property>')
w('          <property name="leftMargin"><number>18</number></property>')
w('          <property name="rightMargin"><number>18</number></property>')
w('          <property name="topMargin"><number>12</number></property>')
w('          <property name="bottomMargin"><number>12</number></property>')
w('          <item>')
w('           <widget class="QLabel" name="pageTitle">')
w('            <property name="text"><string>Point Planning UI</string></property>')
w('            <property name="class" stdset="0"><string notr="true">pageTitle</string></property>')
w('           </widget>')
w('          </item>')
w(spacer(10, "Horizontal", 40, 20))
w('          <item>')
w('           <widget class="QPushButton" name="servoOffButton">')
w('            <property name="text"><string>OFF</string></property>')
w('            <property name="class" stdset="0"><string notr="true">toggleOff</string></property>')
w('            <property name="checkable"><bool>true</bool></property>')
w('            <property name="checked"><bool>true</bool></property>')
w(size("minimumSize", 78, 48, 12))
w('            <attribute name="buttonGroup"><string notr="true">servoPowerGroup</string></attribute>')
w('           </widget>')
w('          </item>')
w('          <item>')
w('           <widget class="QPushButton" name="servoOnButton">')
w('            <property name="text"><string>ON</string></property>')
w('            <property name="class" stdset="0"><string notr="true">toggleOn</string></property>')
w('            <property name="checkable"><bool>true</bool></property>')
w(size("minimumSize", 78, 48, 12))
w('            <attribute name="buttonGroup"><string notr="true">servoPowerGroup</string></attribute>')
w('           </widget>')
w('          </item>')
for text, name, style in [("WS-Calibration", "wsCalibrationButton", "btnBlue"),
                          ("NexSim", "nexSimButton", "btnGrey"),
                          ("Reload Points", "reloadPointsButton", "btnGrey"),
                          ("View Points YAML", "viewYamlButton", "btnBlue")]:
    w('          <item>')
    w(f'           <widget class="QPushButton" name="{name}">')
    w(f'            <property name="text"><string>{text}</string></property>')
    w(f'            <property name="class" stdset="0"><string notr="true">{style}</string></property>')
    w(size("minimumSize", 0, 48, 12))
    w('           </widget>')
    w('          </item>')
w('         </layout>')
w('        </widget>')
w('       </item>')

# three-column body
w('       <item>')
w('        <layout class="QHBoxLayout" name="bodyLayout">')
w('         <property name="spacing"><number>14</number></property>')

# ── column 1: point editor + TF preview
w('         <item>')
w('          <layout class="QVBoxLayout" name="editorColumnLayout">')
w('           <property name="spacing"><number>14</number></property>')
w('           <item>')
w('            <widget class="QFrame" name="pointEditorCard">')
w('             <layout class="QVBoxLayout" name="pointEditorLayout">')
w('              <property name="spacing"><number>10</number></property>')
w('              <property name="leftMargin"><number>16</number></property>')
w('              <property name="rightMargin"><number>16</number></property>')
w('              <property name="topMargin"><number>16</number></property>')
w('              <property name="bottomMargin"><number>16</number></property>')
w('              <item>')
w('               <widget class="QLabel" name="pointEditorTitle">')
w('                <property name="text"><string>Point Editor</string></property>')
w('                <property name="class" stdset="0"><string notr="true">cardTitle</string></property>')
w('               </widget>')
w('              </item>')
w('              <item>')
w('               <layout class="QHBoxLayout" name="nameSeqLayout">')
w('                <property name="spacing"><number>10</number></property>')
w('                <item>')
w('                 <layout class="QVBoxLayout" name="nameFieldLayout">')
w('                  <item>')
w('                   <widget class="QLabel" name="pointNameLabel">')
w('                    <property name="text"><string>POINT NAME</string></property>')
w('                    <property name="class" stdset="0"><string notr="true">fieldLabel</string></property>')
w('                   </widget>')
w('                  </item>')
w('                  <item>')
w('                   <widget class="QLineEdit" name="pointNameInput">')
w('                    <property name="placeholderText"><string>e.g., point-1</string></property>')
w(size("minimumSize", 0, 46, 20))
w('                   </widget>')
w('                  </item>')
w('                 </layout>')
w('                </item>')
w('                <item>')
w('                 <layout class="QVBoxLayout" name="seqFieldLayout">')
w('                  <item>')
w('                   <widget class="QLabel" name="sequenceLabel">')
w('                    <property name="text"><string>SEQUENCE</string></property>')
w('                    <property name="class" stdset="0"><string notr="true">fieldLabel</string></property>')
w('                   </widget>')
w('                  </item>')
w('                  <item>')
w('                   <widget class="QSpinBox" name="sequenceInput">')
w('                    <property name="minimum"><number>1</number></property>')
w('                    <property name="maximum"><number>9999</number></property>')
w(size("minimumSize", 0, 46, 20))
w('                   </widget>')
w('                  </item>')
w('                 </layout>')
w('                </item>')
w('               </layout>')
w('              </item>')
w('              <item>')
w('               <layout class="QHBoxLayout" name="togglesLayout">')
w('                <property name="spacing"><number>10</number></property>')
for text, name in [("Enable TF", "enableTfCheck"), ("Editable", "editableCheck")]:
    w('                <item>')
    w(f'                 <widget class="QCheckBox" name="{name}">')
    w(f'                  <property name="text"><string>{text}</string></property>')
    w(size("minimumSize", 0, 46, 18))
    w('                 </widget>')
    w('                </item>')
w('               </layout>')
w('              </item>')
w('              <item>')
w('               <widget class="QLabel" name="natureLabel">')
w('                <property name="text"><string>NATURE</string></property>')
w('                <property name="class" stdset="0"><string notr="true">fieldLabel</string></property>')
w('               </widget>')
w('              </item>')
w('              <item>')
w('               <widget class="QPlainTextEdit" name="natureInput">')
w('                <property name="placeholderText"><string>e.g., test point</string></property>')
w(size("minimumSize", 0, 86, 16))
w('               </widget>')
w('              </item>')
w('              <item>')
w('               <widget class="QLabel" name="dateTimeLabel">')
w('                <property name="text"><string>DATE TIME</string></property>')
w('                <property name="class" stdset="0"><string notr="true">fieldLabel</string></property>')
w('               </widget>')
w('              </item>')
w('              <item>')
w('               <widget class="QLineEdit" name="dateTimeInput">')
w('                <property name="placeholderText"><string>Auto-generated</string></property>')
w('                <property name="readOnly"><bool>true</bool></property>')
w(size("minimumSize", 0, 46, 16))
w('               </widget>')
w('              </item>')
w('              <item>')
w('               <layout class="QHBoxLayout" name="editorActionsLayout">')
w('                <property name="spacing"><number>10</number></property>')
for text, name, style in [("Add Point", "addPointButton", "btnBlue"),
                          ("Update Point", "updatePointButton", "btnMuted")]:
    w('                <item>')
    w(f'                 <widget class="QPushButton" name="{name}">')
    w(f'                  <property name="text"><string>{text}</string></property>')
    w(f'                  <property name="class" stdset="0"><string notr="true">{style}</string></property>')
    w(size("minimumSize", 0, 52, 18))
    w('                 </widget>')
    w('                </item>')
w('               </layout>')
w('              </item>')
w('             </layout>')
w('            </widget>')
w('           </item>')
w('           <item>')
w('            <widget class="QFrame" name="tfPreviewCard">')
w(size("minimumSize", 0, 250, 13))
w('             <layout class="QVBoxLayout" name="tfPreviewLayout">')
w('              <property name="leftMargin"><number>12</number></property>')
w('              <property name="rightMargin"><number>12</number></property>')
w('              <property name="topMargin"><number>12</number></property>')
w('              <property name="bottomMargin"><number>12</number></property>')
w('              <item>')
w('               <layout class="QHBoxLayout" name="tfPreviewHeaderLayout">')
w('                <item>')
w('                 <widget class="QComboBox" name="tfFrameSelect">')
w(size("minimumSize", 0, 42, 18))
w('                 </widget>')
w('                </item>')
w(spacer(16, "Horizontal", 40, 20))
w('                <item>')
w('                 <widget class="QPushButton" name="tfLockButton">')
w('                  <property name="text"><string>Lock</string></property>')
w('                  <property name="class" stdset="0"><string notr="true">btnRedSmall</string></property>')
w('                  <property name="checkable"><bool>true</bool></property>')
w(size("minimumSize", 62, 42, 18))
w('                 </widget>')
w('                </item>')
w('               </layout>')
w('              </item>')
w('              <item>')
w('               <widget class="QWidget" name="tfCanvas" native="true">')
w('                <property name="class" stdset="0"><string notr="true">tfCanvas</string></property>')
w('               </widget>')
w('              </item>')
w('             </layout>')
w('            </widget>')
w('           </item>')
w('          </layout>')
w('         </item>')

# ── column 2: point list
w('         <item>')
w('          <widget class="QFrame" name="pointListCard">')
w('           <layout class="QVBoxLayout" name="pointListLayout">')
w('            <property name="spacing"><number>12</number></property>')
w('            <property name="leftMargin"><number>16</number></property>')
w('            <property name="rightMargin"><number>16</number></property>')
w('            <property name="topMargin"><number>16</number></property>')
w('            <property name="bottomMargin"><number>16</number></property>')
w('            <item>')
w('             <layout class="QHBoxLayout" name="pointListHeaderLayout">')
w('              <property name="spacing"><number>8</number></property>')
w('              <item>')
w('               <widget class="QLabel" name="pointListTitle">')
w('                <property name="text"><string>Point List</string></property>')
w('                <property name="class" stdset="0"><string notr="true">cardTitle</string></property>')
w('               </widget>')
w('              </item>')
w('              <item>')
w('               <widget class="QLabel" name="pointCountLabel">')
w('                <property name="text"><string>0 points</string></property>')
w('                <property name="class" stdset="0"><string notr="true">countLabel</string></property>')
w('               </widget>')
w('              </item>')
w(spacer(14, "Horizontal", 20, 20))
w('             </layout>')
w('            </item>')
w('            <item>')
w('             <layout class="QHBoxLayout" name="pointListActionsLayout">')
w('              <property name="spacing"><number>8</number></property>')
for text, name, style in [("Manage Access", "manageAccessButton", "btnPurple"),
                          ("Locked", "lockedButton", "btnRed"),
                          ("Delete All", "deleteAllButton", "btnRed"),
                          ("Undo", "undoButton", "btnAmber")]:
    w('              <item>')
    w(f'               <widget class="QPushButton" name="{name}">')
    w(f'                <property name="text"><string>{text}</string></property>')
    w(f'                <property name="class" stdset="0"><string notr="true">{style}</string></property>')
    w(size("minimumSize", 0, 50, 16))
    w('               </widget>')
    w('              </item>')
w('             </layout>')
w('            </item>')
w('            <item>')
w('             <widget class="QListWidget" name="pointList">')
w('              <property name="class" stdset="0"><string notr="true">pointList</string></property>')
w('              <property name="spacing"><number>6</number></property>')
w('              <property name="verticalScrollMode"><enum>QAbstractItemView::ScrollPerPixel</enum></property>')
w('             </widget>')
w('            </item>')
w('           </layout>')
w('          </widget>')
w('         </item>')

# ── column 3: movement control
w('         <item>')
w('          <layout class="QVBoxLayout" name="controlColumnLayout">')
w('           <property name="spacing"><number>14</number></property>')

w('           <item>')
w('            <widget class="QFrame" name="movementControlCard">')
w('             <layout class="QVBoxLayout" name="movementControlLayout">')
w('              <property name="spacing"><number>12</number></property>')
w('              <property name="leftMargin"><number>16</number></property>')
w('              <property name="rightMargin"><number>16</number></property>')
w('              <property name="topMargin"><number>16</number></property>')
w('              <property name="bottomMargin"><number>16</number></property>')
w('              <item>')
w('               <widget class="QLabel" name="movementControlTitle">')
w('                <property name="text"><string>Robot Movement Control</string></property>')
w('                <property name="class" stdset="0"><string notr="true">cardTitle</string></property>')
w('               </widget>')
w('              </item>')
w('              <item>')
w('               <layout class="QHBoxLayout" name="modeRowLayout">')
w('                <property name="spacing"><number>10</number></property>')
w('                <item>')
w('                 <layout class="QHBoxLayout" name="plannerGroupLayout">')
w('                  <property name="spacing"><number>0</number></property>')
w(seg_button("OMPL", "omplButton", True, "plannerGroup", 18))
w(seg_button("PILZ", "pilzButton", False, "plannerGroup", 18))
w('                 </layout>')
w('                </item>')
w('                <item>')
w('                 <layout class="QHBoxLayout" name="modeGroupLayout">')
w('                  <property name="spacing"><number>0</number></property>')
w(seg_button("SafeAuto", "safeAutoButton", False, "modeGroup", 18))
w(seg_button("Jog", "jogModeButton", True, "modeGroup", 18))
w(seg_button("Auto", "autoModeButton", False, "modeGroup", 18))
w('                 </layout>')
w('                </item>')
w('                <item>')
w('                 <widget class="QPushButton" name="jogServoButton">')
w('                  <property name="text"><string>Jog Servo</string></property>')
w('                  <property name="class" stdset="0"><string notr="true">btnBlue</string></property>')
w(size("minimumSize", 0, 50, 18))
w('                 </widget>')
w('                </item>')
w('               </layout>')
w('              </item>')
w('             </layout>')
w('            </widget>')
w('           </item>')

# joint + cartesian
w('           <item>')
w('            <widget class="QFrame" name="jogCard">')
w('             <layout class="QHBoxLayout" name="jogCardLayout">')
w('              <property name="spacing"><number>14</number></property>')
w('              <property name="leftMargin"><number>16</number></property>')
w('              <property name="rightMargin"><number>16</number></property>')
w('              <property name="topMargin"><number>16</number></property>')
w('              <property name="bottomMargin"><number>16</number></property>')

for title, prefix, axes, unit in [
        ("Joint Control", "joint", ["J1", "J2", "J3", "J4", "J5", "J6"], "°"),
        ("Cartesian Control", "cart", ["X", "Y", "Z", "R", "P", "W"], None)]:
    w('              <item>')
    w(f'               <widget class="QFrame" name="{prefix}ControlPanel">')
    w(f'                <property name="class" stdset="0"><string notr="true">innerPanel</string></property>')
    w(f'                <layout class="QVBoxLayout" name="{prefix}ControlLayout">')
    w('                 <property name="spacing"><number>10</number></property>')
    w('                 <property name="leftMargin"><number>14</number></property>')
    w('                 <property name="rightMargin"><number>14</number></property>')
    w('                 <property name="topMargin"><number>14</number></property>')
    w('                 <property name="bottomMargin"><number>14</number></property>')
    w('                 <item>')
    w(f'                  <widget class="QLabel" name="{prefix}ControlTitle">')
    w(f'                   <property name="text"><string>{title}</string></property>')
    w('                   <property name="class" stdset="0"><string notr="true">cardTitle</string></property>')
    w('                  </widget>')
    w('                 </item>')
    for i, ax in enumerate(axes, start=1):
        u = unit if unit else (" cm" if ax in ("X", "Y", "Z") else "°")
        w(jog_row(ax, u, prefix, i, 17))
    w(spacer(17, "Vertical", 20, 10))
    w('                </layout>')
    w('               </widget>')
    w('              </item>')

w('             </layout>')
w('            </widget>')
w('           </item>')

# servo frame
w('           <item>')
w('            <widget class="QFrame" name="servoFrameCard">')
w('             <layout class="QVBoxLayout" name="servoFrameLayout">')
w('              <property name="spacing"><number>12</number></property>')
w('              <property name="leftMargin"><number>16</number></property>')
w('              <property name="rightMargin"><number>16</number></property>')
w('              <property name="topMargin"><number>16</number></property>')
w('              <property name="bottomMargin"><number>16</number></property>')
w('              <item>')
w('               <layout class="QHBoxLayout" name="servoFrameHeaderLayout">')
w('                <item>')
w('                 <widget class="QLabel" name="servoFrameTitle">')
w('                  <property name="text"><string>Servo Frame</string></property>')
w('                  <property name="class" stdset="0"><string notr="true">cardTitle</string></property>')
w('                 </widget>')
w('                </item>')
w(spacer(16, "Horizontal", 40, 20))
w('                <item>')
w('                 <widget class="QLabel" name="activeFrameBadge">')
w('                  <property name="text"><string>ACTIVE  base_link</string></property>')
w('                  <property name="class" stdset="0"><string notr="true">activeBadge</string></property>')
w(size("minimumSize", 0, 46, 18))
w('                 </widget>')
w('                </item>')
w('               </layout>')
w('              </item>')
w('              <item>')
w('               <widget class="QComboBox" name="servoFrameSelect">')
w('                <property name="class" stdset="0"><string notr="true">darkCombo</string></property>')
w(size("minimumSize", 0, 54, 16))
w('               </widget>')
w('              </item>')
w('             </layout>')
w('            </widget>')
w('           </item>')
w(spacer(11, "Vertical", 20, 20))
w('          </layout>')
w('         </item>')

w('        </layout>')
w('       </item>')
w('      </layout>')
w('     </widget>')
w('    </item>')
w('   </layout>')
w('  </widget>')
w('  <widget class="QStatusBar" name="statusbar"/>')
w(' </widget>')

# stretch factors: editor 3 / list 3 / control 5
w(' <buttongroups>')
for g in ["servoPowerGroup", "plannerGroup", "modeGroup"]:
    w(f'  <buttongroup name="{g}"/>')
w(' </buttongroups>')
w(' <resources/>')
w(' <connections/>')
w('</ui>')

open("main_window.ui", "w").write("\n".join(W) + "\n")
print("wrote main_window.ui")
