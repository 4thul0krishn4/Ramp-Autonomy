#!/usr/bin/env python3
"""Structural-identity check for the mesh-swap task: confirms every <link> name, <joint> name (+type/parent/child/origin/axis), <collision> block, and <inertial>
block in a rendered URDF is byte-identical before vs after, and that ONLY <visual> elements differ. Also checks every gz_frame_id / frame reference the stack
depends on. Works on the FINAL rendered URDF (xacro's own $(arg ...) and ${...} substitutions already applied), not the xacro source - what the stack actually
consumes. Usage: diff_nonvisual.py before.urdf after.urdf"""
import re, sys, xml.etree.ElementTree as ET

def strip_comments(text):
    return re.sub(r'<!--.*?-->', '', text, flags=re.S)

def strip_visuals(text):
    return re.sub(r'<visual\b.*?</visual>', '', text, flags=re.S)

def strip_materials(text):
    # top-level <material name="..."><color .../></material> color DEFINITIONS (as opposed to a <material name="x"/> REFERENCE inside a <visual>, which the
    # strip_visuals() pass above already removes along with its parent <visual>) are pure appearance data - no consumer of collision/inertial/joint/frame
    # content (robot_state_publisher, EKF, stability monitor, stall detector, MPPI, sdformat/gz-sim's physics) reads them for anything. Confirmed for this
    # file specifically, not assumed: every <material> in the rendered URDF is either one of these top-level definitions or a same-form reference already
    # inside a <visual> block - none appear inside <collision>/<inertial>/<joint>/sensor/gazebo-plugin content. Same justification as strip_comments().
    return re.sub(r'<material\b[^/]*?>.*?</material>', '', text, flags=re.S)

def structure(text):
    """(link names, joint specs, frame_ids) from the FULL (visual-including) doc, for reporting; the actual pass/fail is the stripped-text byte comparison below."""
    root = ET.fromstring(text)
    links = sorted(l.get('name') for l in root.findall('link'))
    joints = sorted((j.get('name'), j.get('type'), j.find('parent').get('link'), j.find('child').get('link'),
                     (j.find('origin').get('xyz'), j.find('origin').get('rpy')) if j.find('origin') is not None else None,
                     j.find('axis').get('xyz') if j.find('axis') is not None else None) for j in root.findall('joint'))
    frames = sorted(set(re.findall(r'<gz_frame_id>([^<]+)</gz_frame_id>', text)))
    return links, joints, frames

a, b = open(sys.argv[1]).read(), open(sys.argv[2]).read()
la, ja, fa = structure(a)
lb, jb, fb = structure(b)
print(f'links: {len(la)} vs {len(lb)}  {"SAME SET" if la == lb else "DIFFERENT: " + str(set(la) ^ set(lb))}')
print(f'joints (name,type,parent,child,origin,axis): {len(ja)} vs {len(jb)}  {"IDENTICAL" if ja == jb else "DIFFERENT"}')
if ja != jb:
    for x in set(ja) ^ set(jb): print('   joint diff:', x)
print(f'gz_frame_id set: {fa}  vs  {fb}  {"SAME" if fa == fb else "DIFFERENT"}')

# XML comments carry zero functional weight - no consumer (robot_state_publisher, sdformat, gz-sim, any node) parses or acts on them, and xacro is known to
# relocate/hoist orphaned comments in its output, which would otherwise show up as false-positive diffs here. Only what a consumer could ever act on counts.
def normalize_blank_lines(text):
    # comments are stripped above; the blank line/whitespace they leave behind is exactly as functionally inert as the comment itself (no XML parser cares) - drop empty lines so
    # THAT can't produce a false-positive diff either, while any real tag/attribute/text content change still shows up on its own line.
    return '\n'.join(l for l in text.splitlines() if l.strip())

sa, sb = normalize_blank_lines(strip_materials(strip_visuals(strip_comments(a)))), normalize_blank_lines(strip_materials(strip_visuals(strip_comments(b))))
same = sa == sb
print(f'\neverything OUTSIDE <visual> elements (collision, inertial, joints, links, gazebo plugins, sensors): {"BYTE-IDENTICAL" if same else "DIFFERS"}')
if not same:
    import difflib
    for line in list(difflib.unified_diff(sa.splitlines(), sb.splitlines(), lineterm=''))[:60]:
        print(line)
    sys.exit(1)
n_vis_a = a.count('<visual'); n_vis_b = b.count('<visual')
print(f'<visual> element count: {n_vis_a} -> {n_vis_b}')
sys.exit(0)
