#!/usr/bin/env python3
"""Generates worlds/paint_booth.sdf and the MoveIt wall list (paint_cell_moveit_config/config/booth_walls.yaml).
Layout follows the reference drawing with the booth enlarged to 9.5 x 6.5 m after the reach study.
World frame = cell frame (floor z = 0, booth centre at the origin, car front = +X).

    python3 scripts/generate_world.py
"""
import os

T = 0.2            # outer / booth wall thickness
TI = 0.1           # inner divider thickness
H = 3.0            # wall height
BX, BY = 4.75, 3.25            # booth inner half sizes (9.5 x 6.5 m)
ZONE = 2.0                     # entry / exit zone inner width
ROOM = 2.5                     # rooms row inner depth
X_OUT = BX + T + ZONE          # 6.95: inner face of the outer end walls
Y_ROOM0 = BY + T               # 3.45: rooms start
Y_ROOM1 = Y_ROOM0 + ROOM       # 5.95: rooms end
CAR_DOOR_W, CAR_DOOR_H = 3.0, 2.6   # opening for car + skid (skid top 0.5 + car 1.6 = 2.1 m)

WALL = '0.90 0.90 0.88 1'
WALL_OUT = '0.72 0.73 0.74 1'
DOOR = '0.18 0.26 0.36 1'
FRAME = '0.25 0.25 0.27 1'
GLASS = '0.60 0.80 0.95 1'

boxes = []   # (name, (sx, sy, sz), (x, y, z), rgba, transparency)
cylinders = []  # (name, r, h, (x, y, z), rgba, (roll, pitch, yaw))
visual_only = set()


def box(name, x0, x1, y0, y1, z0, z1, rgba=WALL, transparency=0.0):
    boxes.append((name, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), rgba, transparency))


def wall_x(name, x0, x1, y0, y1, openings=(), rgba=WALL):
    """Wall running along X from x0 to x1 (thickness y0..y1) with rectangular openings [(xa, xb, za, zb)]."""
    cuts = sorted(openings)
    x = x0
    for i, (xa, xb, za, zb) in enumerate(cuts):
        if xa > x:
            box(f'{name}_{i}a', x, xa, y0, y1, 0, H, rgba)
        if za > 0:
            box(f'{name}_{i}below', xa, xb, y0, y1, 0, za, rgba)
        if zb < H:
            box(f'{name}_{i}above', xa, xb, y0, y1, zb, H, rgba)
        x = xb
    if x < x1:
        box(f'{name}_end', x, x1, y0, y1, 0, H, rgba)


def wall_y(name, x0, x1, y0, y1, openings=(), rgba=WALL):
    """Wall running along Y from y0 to y1 (thickness x0..x1) with openings [(ya, yb, za, zb)]."""
    cuts = sorted(openings)
    y = y0
    for i, (ya, yb, za, zb) in enumerate(cuts):
        if ya > y:
            box(f'{name}_{i}a', x0, x1, y, ya, 0, H, rgba)
        if za > 0:
            box(f'{name}_{i}below', x0, x1, ya, yb, 0, za, rgba)
        if zb < H:
            box(f'{name}_{i}above', x0, x1, ya, yb, zb, H, rgba)
        y = yb
    if y < y1:
        box(f'{name}_end', x0, x1, y, y1, 0, H, rgba)


def door_x(name, xc, y0, y1, w=1.0, h=2.1):
    """Closed personnel door (panel + frame) in a wall along X."""
    box(f'{name}_panel', xc - w / 2, xc + w / 2, y0 + 0.06, y1 - 0.06, 0, h, DOOR)
    box(f'{name}_frame_l', xc - w / 2 - 0.06, xc - w / 2, y0 - 0.01, y1 + 0.01, 0, h + 0.06, FRAME)
    box(f'{name}_frame_r', xc + w / 2, xc + w / 2 + 0.06, y0 - 0.01, y1 + 0.01, 0, h + 0.06, FRAME)
    box(f'{name}_frame_t', xc - w / 2, xc + w / 2, y0 - 0.01, y1 + 0.01, h, h + 0.06, FRAME)


def window_x(name, xa, xb, za, zb, y0, y1):
    box(f'{name}_glass', xa, xb, y0 + 0.08, y1 - 0.08, za, zb, GLASS, 0.6)
    box(f'{name}_sill', xa - 0.05, xb + 0.05, y0 - 0.03, y1 + 0.03, za - 0.05, za, FRAME)
    box(f'{name}_head', xa - 0.05, xb + 0.05, y0 - 0.01, y1 + 0.01, zb, zb + 0.05, FRAME)


# floors
box('floor_building', -X_OUT - T, X_OUT + T, -BY - T, Y_ROOM1 + T, -0.02, 0.0, '0.55 0.55 0.55 1')
box('floor_booth', -BX, BX, -BY, BY, 0.0, 0.002, '0.38 0.40 0.42 1')
for s, nm in ((-1, 'entry'), (1, 'exit')):   # yellow safety lines across the car doorways
    x = s * (BX + T / 2)
    box(f'floor_line_{nm}', x - 0.05, x + 0.05, -CAR_DOOR_W / 2, CAR_DOOR_W / 2, 0.002, 0.004, '0.95 0.80 0.10 1')

# outer walls
win_z = (1.0, 2.2)
wall_x('wall_south', -X_OUT - T, X_OUT + T, -BY - T, -BY, rgba=WALL_OUT, openings=[
    (-X_OUT + 0.4, -X_OUT + 1.6, *win_z),        # observation window, entry zone
    (X_OUT - 1.6, X_OUT - 0.4, *win_z)])          # observation window, exit zone
window_x('window_entry', -X_OUT + 0.4, -X_OUT + 1.6, *win_z, -BY - T, -BY)
window_x('window_exit', X_OUT - 1.6, X_OUT - 0.4, *win_z, -BY - T, -BY)
wall_x('wall_north', -X_OUT - T, X_OUT + T, Y_ROOM1, Y_ROOM1 + T, rgba=WALL_OUT)
car_door_y = [(-CAR_DOOR_W / 2, CAR_DOOR_W / 2, 0.0, CAR_DOOR_H)]
wall_y('wall_west', -X_OUT - T, -X_OUT, -BY, Y_ROOM1, car_door_y, rgba=WALL_OUT)   # car entry
wall_y('wall_east', X_OUT, X_OUT + T, -BY, Y_ROOM1, car_door_y, rgba=WALL_OUT)     # car exit

# booth walls
wall_y('booth_wall_west', -BX - T, -BX, -BY, BY, car_door_y)
wall_y('booth_wall_east', BX, BX + T, -BY, BY, car_door_y)
# booth / rooms wall (+Y): doors from the rooms and the control room observation window
ctrl_win = (-4.4, -2.8, 1.0, 2.1)
wall_x('wall_rooms', -X_OUT, X_OUT, BY, BY + T, openings=[
    (-6.45, -5.45, 0.0, 2.1),     # control room door  -> entry zone
    (ctrl_win),                   # control room window -> booth
    (-1.1, -0.1, 0.0, 2.1),       # paint mixing door  -> booth
    (5.45, 6.45, 0.0, 2.1)])      # equipment door     -> exit zone
door_x('door_control', -5.95, BY, BY + T)
window_x('window_control', ctrl_win[0], ctrl_win[1], ctrl_win[2], ctrl_win[3], BY, BY + T)
door_x('door_mixing', -0.6, BY, BY + T)
door_x('door_equipment', 5.95, BY, BY + T)

# rooms row dividers
X_DIV1, X_DIV2 = -2.55, 1.15          # control 4.4 m | mixing 3.6 m | equipment 5.7 m
wall_y('divider_1', X_DIV1, X_DIV1 + TI, Y_ROOM0, Y_ROOM1)
wall_y('divider_2', X_DIV2, X_DIV2 + TI, Y_ROOM0, Y_ROOM1)

# room contents
# control room: operator desk with two screens, electrical cabinet
box('ctrl_desk', -5.6, -3.6, 4.6, 5.3, 0, 0.75, '0.45 0.35 0.25 1')
box('ctrl_screen_1', -5.2, -4.6, 5.05, 5.1, 0.80, 1.20, '0.05 0.05 0.06 1')
box('ctrl_screen_2', -4.5, -3.9, 5.05, 5.1, 0.80, 1.20, '0.05 0.05 0.06 1')
box('ctrl_cabinet', -6.85, -6.25, 5.35, 5.9, 0, 2.0, '0.55 0.58 0.62 1')
box('ctrl_cabinet_2', -3.4, -2.7, 5.35, 5.9, 0, 2.0, '0.55 0.58 0.62 1')
# paint mixing room: drums, mixing table, shelf
for i, (x, y, c) in enumerate([(-2.0, 5.5, '0.75 0.10 0.10 1'), (-1.35, 5.5, '0.10 0.25 0.70 1'),
                               (-0.7, 5.5, '0.95 0.95 0.95 1'), (-2.0, 4.85, '0.15 0.15 0.15 1')]):
    cylinders.append((f'paint_drum_{i}', 0.29, 0.88, (x, y, 0.44), c, (0, 0, 0)))
box('mix_table', -0.2, 0.9, 4.9, 5.6, 0, 0.9, '0.60 0.62 0.65 1')
box('mix_shelf', 0.2, 1.1, 3.6, 4.0, 0, 1.8, '0.40 0.42 0.45 1')
# equipment room: air supply unit, compressor, cabinets
box('equip_air_unit', 1.5, 3.3, 4.4, 5.85, 0, 2.4, '0.70 0.72 0.75 1')
box('equip_compressor', 3.8, 5.0, 4.9, 5.8, 0, 1.2, '0.20 0.40 0.30 1')
box('equip_cabinet_1', 5.4, 6.0, 5.35, 5.9, 0, 2.0, '0.55 0.58 0.62 1')
box('equip_cabinet_2', 6.1, 6.85, 5.35, 5.9, 0, 2.0, '0.55 0.58 0.62 1')

# lights
lights = [(x, y) for x in (-3.0, 0.0, 3.0) for y in (-1.6, 1.6)]


def f(v):
    return ' '.join(f'{a:.4g}' for a in v)


def sdf():
    out = []
    w = out.append
    w('<?xml version="1.0"?>')
    w('<!-- GENERATED by scripts/generate_world.py - edit the script, not this file -->')
    w('<sdf version="1.9">')
    w('  <world name="paint_booth">')
    # 4 ms steps: enough for position-controlled joints, 4x cheaper than the default
    w('    <physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>')
    for p, n in (('gz-sim-physics-system', 'gz::sim::systems::Physics'),
                 ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
                 ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster')):
        w(f'    <plugin filename="{p}" name="{n}"/>')
    w('    <gravity>0 0 -9.81</gravity>')
    w('    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.75 0.80 0.85 1</background>'
      '<shadows>true</shadows><grid>false</grid></scene>')
    w('    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 12 0 0 0</pose>'
      '<diffuse>0.7 0.7 0.7 1</diffuse><specular>0.2 0.2 0.2 1</specular>'
      '<direction>-0.4 0.3 -0.9</direction></light>')
    for i, (x, y) in enumerate(lights):
        w(f'    <light type="point" name="booth_light_{i}"><pose>{x} {y} 2.9 0 0 0</pose><cast_shadows>false</cast_shadows>'
          '<diffuse>0.55 0.55 0.52 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
          '<attenuation><range>8</range><constant>0.6</constant><linear>0.08</linear><quadratic>0.01</quadratic></attenuation></light>')
    w('    <model name="ground"><static>true</static><link name="link">'
      '<collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>'
      '<visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>'
      '<material><ambient>0.35 0.37 0.35 1</ambient><diffuse>0.35 0.37 0.35 1</diffuse></material></visual>'
      '</link><pose>0 0 -0.021 0 0 0</pose></model>')
    w('    <model name="paint_booth_building"><static>true</static><link name="link">')
    for name, size, pos, rgba, tr in boxes:
        geo = f'<geometry><box><size>{f(size)}</size></box></geometry>'
        mat = f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.1 0.1 0.1 1</specular></material>'
        trn = f'<transparency>{tr}</transparency><cast_shadows>false</cast_shadows>' if tr else ''
        if name not in visual_only:
            w(f'      <collision name="{name}_c"><pose>{f(pos)} 0 0 0</pose>{geo}</collision>')
        w(f'      <visual name="{name}"><pose>{f(pos)} 0 0 0</pose>{geo}{mat}{trn}</visual>')
    for name, r, h, pos, rgba, rpy in cylinders:
        geo = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
        mat = f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.3 0.3 0.3 1</specular></material>'
        if name not in visual_only:
            w(f'      <collision name="{name}_c"><pose>{f(pos)} {f(rpy)}</pose>{geo}</collision>')
        w(f'      <visual name="{name}"><pose>{f(pos)} {f(rpy)}</pose>{geo}{mat}</visual>')
    w('    </link></model>')
    w('  </world>')
    w('</sdf>')
    return '\n'.join(out) + '\n'


def moveit_walls():
    """Booth walls for the MoveIt planning scene (frame: world).
    Side walls are full slabs (personnel doors closed); the end walls have the car doorway, so the
    car + skid on the conveyor can be planned through it."""
    d, h = CAR_DOOR_W / 2, CAR_DOOR_H
    walls = [
        ('booth_wall_north', (2 * BX + 2 * T, T, H), (0, BY + T / 2, H / 2)),
        ('booth_wall_south', (2 * BX + 2 * T, T, H), (0, -BY - T / 2, H / 2)),
    ]
    for s, nm in ((-1, 'west'), (1, 'east')):
        x = s * (BX + T / 2)
        walls += [
            (f'booth_wall_{nm}_a', (T, BY - d, H), (x, -(BY + d) / 2, H / 2)),
            (f'booth_wall_{nm}_b', (T, BY - d, H), (x, (BY + d) / 2, H / 2)),
            (f'booth_wall_{nm}_lintel', (T, 2 * d, H - h), (x, 0, (H + h) / 2)),
        ]
    lines = ['# GENERATED by scripts/generate_world.py - booth walls added to the MoveIt planning scene',
             '# (inner faces: x = +-%.2f, y = +-%.2f; height %.1f m; car doorways %.1f x %.1f m; frame: world)'
             % (BX, BY, H, CAR_DOOR_W, CAR_DOOR_H), 'walls:']
    for n, sz, p in walls:
        lines.append(f'  - {{name: {n}, size: [{sz[0]:.3f}, {sz[1]:.3f}, {sz[2]:.3f}], position: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]}}')
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(os.path.join(pkg, 'worlds'), exist_ok=True)
    open(os.path.join(pkg, 'worlds', 'paint_booth.sdf'), 'w').write(sdf())
    walls = os.path.join(os.path.dirname(pkg), 'paint_cell_moveit_config', 'config', 'booth_walls.yaml')
    open(walls, 'w').write(moveit_walls())
    print(f'{len(boxes)} boxes, {len(cylinders)} cylinders, {len(lights)} lights '
          f'-> worlds/paint_booth.sdf, paint_cell_moveit_config/config/booth_walls.yaml')
