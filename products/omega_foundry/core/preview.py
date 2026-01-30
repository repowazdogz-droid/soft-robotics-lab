"""
OMEGA Foundry — 3D preview from MJCF.
Parse MJCF to extract geometry (boxes, capsules, spheres, cylinders); render via Three.js (HTML) or matplotlib.
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional

# Default RGBA when not specified
_DEFAULT_RGBA = (0.6, 0.6, 0.7, 1.0)


def _parse_vec(s: Optional[str], dim: int = 3) -> List[float]:
    if not s or not s.strip():
        return [0.0] * dim
    parts = s.strip().split()
    out = []
    for i, p in enumerate(parts):
        if i >= dim:
            break
        try:
            out.append(float(p))
        except ValueError:
            out.append(0.0)
    while len(out) < dim:
        out.append(0.0)
    return out[:dim]


def _parse_rgba(rgba_str: Optional[str]) -> tuple:
    if not rgba_str or not rgba_str.strip():
        return _DEFAULT_RGBA
    v = _parse_vec(rgba_str, 4)
    if len(v) < 4:
        v.extend([1.0] * (4 - len(v)))
    return (v[0], v[1], v[2], v[3] if len(v) > 3 else 1.0)


def _body_pos_chain(body_el: Optional[ET.Element]) -> List[float]:
    """Accumulate pos from body up to root."""
    pos = [0.0, 0.0, 0.0]
    current = body_el
    while current is not None:
        p = current.get("pos")
        if p:
            v = _parse_vec(p, 3)
            pos[0] += v[0]
            pos[1] += v[1]
            pos[2] += v[2]
        current = current.find("..") if hasattr(current, "find") else None
    # ElementTree has no parent by default; we traverse from root and accumulate
    return pos


def _find_parent_map(root: ET.Element) -> Dict[ET.Element, Optional[ET.Element]]:
    """Build parent map for body elements."""
    parent_map: Dict[ET.Element, Optional[ET.Element]] = {}

    def walk(el: ET.Element, parent: Optional[ET.Element]) -> None:
        parent_map[el] = parent
        for child in el:
            walk(child, el)

    walk(root, None)
    return parent_map


def _accumulate_body_pos(body_el: ET.Element, parent_map: Dict[ET.Element, Optional[ET.Element]]) -> List[float]:
    pos = [0.0, 0.0, 0.0]
    current: Optional[ET.Element] = body_el
    while current is not None:
        p = current.get("pos")
        if p:
            v = _parse_vec(p, 3)
            pos[0] += v[0]
            pos[1] += v[1]
            pos[2] += v[2]
        current = parent_map.get(current)
    return pos


def mjcf_to_geometry(mjcf: str) -> List[Dict[str, Any]]:
    """
    Parse MJCF string and extract geometry list for preview.
    Returns list of dicts: {type, size, pos, color/rgba, fromto (for capsule)}.
    """
    geometries: List[Dict[str, Any]] = []
    try:
        root = ET.fromstring(mjcf)
    except ET.ParseError:
        return geometries

    world = root.find("worldbody")
    if world is None:
        return geometries

    parent_map = _find_parent_map(root)

    def process_geom(geom_el: ET.Element, body_pos: List[float]) -> None:
        geom_type = (geom_el.get("type") or "sphere").lower()
        size_str = geom_el.get("size") or geom_el.get("halfextent") or "0.01"
        pos_str = geom_el.get("pos") or "0 0 0"
        rgba_str = geom_el.get("rgba")
        fromto_str = geom_el.get("fromto")

        pos = _parse_vec(pos_str, 3)
        pos[0] += body_pos[0]
        pos[1] += body_pos[1]
        pos[2] += body_pos[2]
        color = _parse_rgba(rgba_str)

        size_vec = _parse_vec(size_str, 3)
        if len(size_vec) == 1:
            size_vec = [size_vec[0], size_vec[0], size_vec[0]]

        if geom_type == "plane":
            # Skip ground plane or make it very thin for display
            return
        if geom_type == "box":
            geometries.append({"type": "box", "size": size_vec, "pos": pos, "rgba": color})
        elif geom_type == "sphere":
            r = size_vec[0] if size_vec else 0.01
            geometries.append({"type": "sphere", "size": [r, r, r], "pos": pos, "rgba": color})
        elif geom_type == "cylinder":
            # MuJoCo cylinder size = (radius, halfheight)
            r = size_vec[0] if size_vec else 0.01
            h = size_vec[1] * 2 if len(size_vec) > 1 else r * 2
            geometries.append({"type": "cylinder", "size": [r, h], "pos": pos, "rgba": color})
        elif geom_type == "capsule":
            if fromto_str:
                f = _parse_vec(fromto_str, 6)
                from_pt = f[0:3]
                to_pt = f[3:6]
                mid = [(from_pt[i] + to_pt[i]) / 2 for i in range(3)]
                mid[0] += pos[0]
                mid[1] += pos[1]
                mid[2] += pos[2]
                length = sum((to_pt[i] - from_pt[i]) ** 2 for i in range(3)) ** 0.5
                radius = size_vec[0] if size_vec else 0.002
                geometries.append({"type": "capsule", "size": [radius, length], "pos": mid, "rgba": color})
            else:
                r = size_vec[0] if size_vec else 0.01
                geometries.append({"type": "capsule", "size": [r, r * 2], "pos": pos, "rgba": color})
        else:
            geometries.append({"type": "box", "size": size_vec, "pos": pos, "rgba": color})

    def walk_bodies(el: ET.Element, body_pos: List[float]) -> None:
        pos_str = el.get("pos") or "0 0 0"
        delta = _parse_vec(pos_str, 3)
        current_pos = [body_pos[0] + delta[0], body_pos[1] + delta[1], body_pos[2] + delta[2]]
        for geom in el.findall("geom"):
            process_geom(geom, current_pos)
        for body in el.findall("body"):
            walk_bodies(body, current_pos)

    # worldbody may have geom (e.g. ground) and body children
    for geom in world.findall("geom"):
        process_geom(geom, [0.0, 0.0, 0.0])
    for body in world.findall("body"):
        walk_bodies(body, [0.0, 0.0, 0.0])

    return geometries


def generate_preview_html(geometries: List[Dict[str, Any]], height: int = 400) -> str:
    """
    Generate self-contained HTML with Three.js to render geometries.
    Suitable for st.components.v1.html(html, height=400).
    """
    if not geometries:
        return """<!DOCTYPE html><html><body style="margin:0;background:#1a1a2e;color:#888;font-family:sans-serif;display:flex;align-items:center;justify-content:center;height:100%;">No geometry</body></html>"""

    def rgba_hex(g: Dict) -> str:
        r, g, b, a = g.get("rgba", _DEFAULT_RGBA)
        return f"#{int(r*255):02x}{int(g*255):02x}{int(b*255):02x}"

    # Build Three.js scene as JSON-like arrays for JS
    objects_js = []
    for g in geometries:
        t = g.get("type", "box")
        pos = g.get("pos", [0, 0, 0])
        rgba = g.get("rgba", _DEFAULT_RGBA)
        color = rgba_hex(g)
        size = g.get("size", [0.01, 0.01, 0.01])
        if t == "box":
            objects_js.append(f'{{type:"box",pos:[{pos[0]},{pos[1]},{pos[2]}],size:[{size[0]},{size[1]},{size[2]}],color:"{color}"}}')
        elif t == "sphere":
            r = size[0]
            objects_js.append(f'{{type:"sphere",pos:[{pos[0]},{pos[1]},{pos[2]}],radius:{r},color:"{color}"}}')
        elif t == "cylinder":
            r = size[0]
            h = size[1]
            objects_js.append(f'{{type:"cylinder",pos:[{pos[0]},{pos[1]},{pos[2]}],radius:{r},height:{h},color:"{color}"}}')
        elif t == "capsule":
            r = size[0]
            h = size[1]
            objects_js.append(f'{{type:"capsule",pos:[{pos[0]},{pos[1]},{pos[2]}],radius:{r},height:{h},color:"{color}"}}')
        else:
            objects_js.append(f'{{type:"box",pos:[{pos[0]},{pos[1]},{pos[2]}],size:[{size[0]},{size[1]},{size[2]}],color:"{color}"}}')

    objs_str = "[" + ",".join(objects_js) + "]"

    html = f"""<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <style>body {{ margin: 0; background: #1a1a2e; overflow: hidden; }}</style>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
</head>
<body>
  <script>
(function() {{
  const objs = {objs_str};
  const w = window.innerWidth || 400, h = window.innerHeight || 400;
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(50, w/h, 0.01, 100);
  camera.position.set(0.15, 0.15, 0.2);
  camera.lookAt(0, 0, 0.08);
  const renderer = new THREE.WebGLRenderer({{ antialias: true, alpha: true }});
  renderer.setSize(w, h);
  renderer.setClearColor(0x1a1a2e, 1);
  document.body.appendChild(renderer.domElement);

  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(1, 1, 1);
  scene.add(dirLight);
  scene.add(new THREE.AmbientLight(0xffffff, 0.4));

  function addBox(o) {{
    const g = new THREE.BoxGeometry(o.size[0]*2, o.size[1]*2, o.size[2]*2);
    const m = new THREE.MeshPhongMaterial({{ color: o.color }});
    const mesh = new THREE.Mesh(g, m);
    mesh.position.set(o.pos[0], o.pos[1], o.pos[2]);
    scene.add(mesh);
  }}
  function addSphere(o) {{
    const g = new THREE.SphereGeometry(o.radius, 16, 12);
    const m = new THREE.MeshPhongMaterial({{ color: o.color }});
    const mesh = new THREE.Mesh(g, m);
    mesh.position.set(o.pos[0], o.pos[1], o.pos[2]);
    scene.add(mesh);
  }}
  function addCylinder(o) {{
    const g = new THREE.CylinderGeometry(o.radius, o.radius, o.height, 24);
    const m = new THREE.MeshPhongMaterial({{ color: o.color }});
    const mesh = new THREE.Mesh(g, m);
    mesh.position.set(o.pos[0], o.pos[1], o.pos[2]);
    scene.add(mesh);
  }}
  function addCapsule(o) {{
    const r = o.radius, h = Math.max(0.001, o.height - 2*r);
    const cyl = new THREE.CylinderGeometry(r, r, h, 16);
    const sph = new THREE.SphereGeometry(r, 16, 12);
    const mat = new THREE.MeshPhongMaterial({{ color: o.color }});
    const mesh = new THREE.Group();
    const c = new THREE.Mesh(cyl, mat);
    c.position.y = 0;
    mesh.add(c);
    const s1 = new THREE.Mesh(sph, mat);
    s1.position.y = h/2;
    mesh.add(s1);
    const s2 = new THREE.Mesh(sph, mat);
    s2.position.y = -h/2;
    mesh.add(s2);
    mesh.position.set(o.pos[0], o.pos[1], o.pos[2]);
    scene.add(mesh);
  }}

  objs.forEach(function(o) {{
    if (o.type === "box") addBox(o);
    else if (o.type === "sphere") addSphere(o);
    else if (o.type === "cylinder") addCylinder(o);
    else if (o.type === "capsule") addCapsule(o);
    else addBox(o);
  }});

  function animate() {{
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
  }}
  animate();
}})();
  </script>
</body>
</html>"""
    return html


def generate_preview_figure(geometries: List[Dict[str, Any]]):  # -> Optional[matplotlib.figure.Figure]
    """
    Generate matplotlib 3D figure for geometries. Returns Figure or None if matplotlib unavailable.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        import numpy as np
    except ImportError:
        return None

    if not geometries:
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        ax.set_xlim(-0.05, 0.05)
        ax.set_ylim(-0.05, 0.05)
        ax.set_zlim(0, 0.1)
        return fig

    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111, projection="3d")

    for g in geometries:
        t = g.get("type", "box")
        pos = g.get("pos", [0, 0, 0])
        rgba = g.get("rgba", _DEFAULT_RGBA)
        size = g.get("size", [0.01, 0.01, 0.01])
        color = (rgba[0], rgba[1], rgba[2])

        if t == "box":
            x, y, z = pos[0], pos[1], pos[2]
            sx, sy, sz = size[0], size[1], size[2]
            # Draw box as 6 faces (front/back, left/right, top/bottom)
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
            dx, dy, dz = 2 * sx, 2 * sy, 2 * sz
            xx = [x - sx, x + sx]
            yy = [y - sy, y + sy]
            zz = [z - sz, z + sz]
            verts = [
                [[xx[0], yy[0], zz[0]], [xx[1], yy[0], zz[0]], [xx[1], yy[1], zz[0]], [xx[0], yy[1], zz[0]]],
                [[xx[0], yy[0], zz[1]], [xx[1], yy[0], zz[1]], [xx[1], yy[1], zz[1]], [xx[0], yy[1], zz[1]]],
                [[xx[0], yy[0], zz[0]], [xx[1], yy[0], zz[0]], [xx[1], yy[0], zz[1]], [xx[0], yy[0], zz[1]]],
                [[xx[1], yy[1], zz[0]], [xx[0], yy[1], zz[0]], [xx[0], yy[1], zz[1]], [xx[1], yy[1], zz[1]]],
                [[xx[0], yy[0], zz[0]], [xx[0], yy[1], zz[0]], [xx[0], yy[1], zz[1]], [xx[0], yy[0], zz[1]]],
                [[xx[1], yy[0], zz[0]], [xx[1], yy[1], zz[0]], [xx[1], yy[1], zz[1]], [xx[1], yy[0], zz[1]]],
            ]
            ax.add_collection3d(Poly3DCollection(verts, facecolors=color, alpha=0.9, edgecolors="k", linewidths=0.2))
        elif t == "sphere":
            u = np.linspace(0, 2 * np.pi, 12)
            v = np.linspace(0, np.pi, 10)
            r = size[0]
            x = pos[0] + r * np.outer(np.cos(u), np.sin(v))
            y = pos[1] + r * np.outer(np.sin(u), np.sin(v))
            z = pos[2] + r * np.outer(np.ones(np.size(u)), np.cos(v))
            ax.plot_surface(x, y, z, color=color, alpha=0.9)
        elif t == "cylinder" or t == "capsule":
            r = size[0]
            h = size[1] if len(size) > 1 else r * 2
            u = np.linspace(0, 2 * np.pi, 24)
            z_lin = np.linspace(-h / 2, h / 2, 2)
            x = pos[0] + r * np.outer(np.cos(u), np.ones_like(z_lin))
            y = pos[1] + r * np.outer(np.sin(u), np.ones_like(z_lin))
            z = pos[2] + np.outer(np.ones_like(u), z_lin)
            ax.plot_surface(x, y, z, color=color, alpha=0.9)

    margin = 0.05
    all_x = [p[0] for g in geometries for p in [g.get("pos", [0, 0, 0])]]
    all_y = [p[1] for g in geometries for p in [g.get("pos", [0, 0, 0])]]
    all_z = [p[2] for g in geometries for p in [g.get("pos", [0, 0, 0])]]
    if all_x:
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        ax.set_zlim(max(0, min(all_z) - margin), max(all_z) + margin)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    return fig
